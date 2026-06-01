#include <Arduino.h>
#include <SPI.h>
#include "ADS131M08.h"
#include <esp_mac.h>

// Включаем поддержку нативного USB HID геймпада
#include "USB.h"
#include "USBHIDGamepad.h"

// Включаем поддержку NimBLE для BleGamepad
#define USE_NIMBLE 
#include <BleGamepad.h>
#include <algorithm>

// Подключение модулей NimBLE
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEService.h>
#include <NimBLECharacteristic.h>
#include <NimBLEAdvertising.h>

// ===================== НАСТРОЙКИ СЕТИ И ЖЕЛЕЗА =====================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DATA_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CMD_CHAR_UUID       "c0de0001-36e1-4688-b7f5-ea07361b26a8" // Прозрачный мост

#define PIN_CS1    D0  
#define PIN_DRDY1  D1  // Прерывание берем только с первого АЦП
#define PIN_RESET  D2  // Общий сброс для обоих чипов
#define PIN_CS2    D3  
#define PIN_SCLK   D8  
#define PIN_MISO   D9  
#define PIN_MOSI   D10 
#define PIN_CLKOUT D6

#define MAX_CHANNELS 16
#define BUF_SIZE 256 

#define RADIUS_EXT 11.5f
#define RADIUS_INT 6.0f

const float UV_SCALE = (1.2f / 4.0f / 8388607.0f) * 1000000.0f;

const float ANGLES_8[8] = {-72.0f, -36.0f, 36.0f, 72.0f, 108.0f, 144.0f, -144.0f, -108.0f};
const float COORDS_16_X[16] = {10.14f, 7.43f, 2.75f, 2.72f, -2.72f, -2.75f, -7.42f, -10.14f, -10.14f, -7.43f, -2.75f, -2.72f, 2.72f, 2.75f, 7.43f, 10.14f};
const float COORDS_16_Y[16] = {-2.72f, -7.43f, -4.77f, -10.15f, -10.14f, -4.77f, -7.42f, -2.73f, 2.72f, 7.43f, 4.76f, 10.14f, 10.15f, 4.77f, 7.42f, 2.71f};

__attribute__((aligned(16))) float EX[MAX_CHANNELS];
__attribute__((aligned(16))) float EY[MAX_CHANNELS];
volatile int activeChannels = 16; 

ADS131M08 adc1;
ADS131M08 adc2;

// Функция безопасного получения уникального MAC-имени устройства
std::string getMacName() {
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_BT) != ESP_OK) {
        esp_efuse_mac_get_default(mac);
    }
    char name[32];
    snprintf(name, sizeof(name), "FE-Gamepad-%02X%02X", mac[4], mac[5]);
    return std::string(name);
}

// Статическая инициализация Bluetooth геймпада (безопасный boot)
BleGamepad bleGamepad("FreeEEG Gamepad", "NeuroIDSS", 100);
BleGamepadConfiguration bleGamepadConfig;

// Инициализация нативного USB HID геймпада
USBHIDGamepad Gamepad;

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pDataCharacteristic = nullptr;
NimBLECharacteristic* pCmdCharacteristic = nullptr;

volatile int eeg_head = 0;

__attribute__((aligned(16))) float eegRing[MAX_CHANNELS][BUF_SIZE];
__attribute__((aligned(16))) float localEEG[MAX_CHANNELS][BUF_SIZE];
__attribute__((aligned(16))) float re[MAX_CHANNELS][BUF_SIZE];
__attribute__((aligned(16))) float im[MAX_CHANNELS][BUF_SIZE];
__attribute__((aligned(16))) float norm_re[MAX_CHANNELS][37];
__attribute__((aligned(16))) float norm_im[MAX_CHANNELS][37];

uint8_t customPacket[51];
uint8_t sampleCounter = 0;

// === ФЛАГИ ПРОЗРАЧНОГО МОСТА ===
volatile bool has_pending_read = false;
volatile bool has_pending_write = false;
volatile uint8_t  reg_addr = 0;
volatile uint16_t reg_val = 0;
volatile uint16_t reg_mask = 0;
volatile bool use_mask = false;

// ===================== ФИЗИКА ИЗ ДОНГЛА =====================
float persistence = 0.0f;
float lastTX = 0.0f, lastTY = 0.0f;

float smooth_lx = 0.0f, smooth_ly = 0.0f, smooth_rx = 0.0f;

// Дескрипторы задач FreeRTOS
TaskHandle_t ADCTaskHandle = NULL;
TaskHandle_t DSPTaskHandle = NULL;
SemaphoreHandle_t bufferMutex = NULL;

// ===================== ПРЕДРАСЧЕТЫ FFT (LUT) =====================
uint16_t revLUT[BUF_SIZE];
float wReLUT[BUF_SIZE / 2];
float wImLUT[BUF_SIZE / 2];

void initDSP() {
    for (int i = 0; i < BUF_SIZE; i++) {
        int x = i, res = 0;
        for (int k = 0; k < 8; k++) { res = (res << 1) | (x & 1); x >>= 1; }
        revLUT[i] = res;
    }
    for (int i = 0; i < BUF_SIZE / 2; i++) {
        wReLUT[i] = cosf(-PI * i / (BUF_SIZE / 2.0f));
        wImLUT[i] = sinf(-PI * i / (BUF_SIZE / 2.0f));
    }
}

void fast_fft(float* vR, float* vI) {
    for (int i = 0; i < BUF_SIZE; i++) {
        if (revLUT[i] > i) {
            std::swap(vR[i], vR[revLUT[i]]);
            std::swap(vI[i], vI[revLUT[i]]);
        }
    }
    for (int s = 2; s <= BUF_SIZE; s <<= 1) {
        int m = s >> 1;
        int step = (BUF_SIZE / 2) / m;
        for (int b = 0; b < BUF_SIZE; b += s) {
            for (int j = 0; j < m; j++) {
                float wr = wReLUT[j * step], wi = wImLUT[j * step];
                int u = b + j, v = u + m;
                float tr = wr * vR[v] - wi * vI[v];
                float ti = wr * vI[v] + wi * vR[v];
                vR[v] = vR[u] - tr; vI[v] = vI[u] - ti;
                vR[u] += tr; vI[u] += ti;
            }
        }
    }
}

// Прерывание DRDY теперь мгновенно будит выделенную задачу захвата на Ядре 1
void IRAM_ATTR onDrdy() { 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (ADCTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(ADCTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

// === КОЛЛБЕКИ КОМАНД С ВЕБ-ИНТЕРФЕЙСА ===
class CmdCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
        std::string rx = pChar->getValue();
        if (rx.length() == 1) {
            reg_addr = rx[0];
            has_pending_read = true;
        } else if (rx.length() == 3) {
            reg_addr = rx[0];
            reg_val = (rx[1] << 8) | rx[2];
            use_mask = false;
            has_pending_write = true;
        } else if (rx.length() == 5) {
            reg_addr = rx[0];
            reg_val = (rx[1] << 8) | rx[2];
            reg_mask = (rx[3] << 8) | rx[4];
            use_mask = true;
            has_pending_write = true;
        }
    }
};

// ===================== МЕТОДЫ УПРАВЛЕНИЯ РОТАЦИЕЙ РЕКЛАМЫ =====================
void startAdvertisingEEG() {
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    if (pAdvertising) {
        pAdvertising->stop();
        
        NimBLEAdvertisementData adv;
        adv.setFlags(0x06); // General Discoverable, BR/EDR Not Supported
        adv.setCompleteServices(NimBLEUUID(SERVICE_UUID));
        pAdvertising->setAdvertisementData(adv);
        
        NimBLEAdvertisementData scanResp; 
        pAdvertising->setScanResponseData(scanResp);
        
        pAdvertising->enableScanResponse(false);
        pAdvertising->start();
        Serial.println("[ADV] Advertising only Raw EEG Service.");
    }
}

void startAdvertisingGamepad() {
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    if (pAdvertising) {
        pAdvertising->stop();
        
        NimBLEAdvertisementData adv;
        adv.setFlags(0x06);
        adv.setCompleteServices(NimBLEUUID("1812")); // HID Service
        adv.setAppearance(0x03C4); // Gamepad
        pAdvertising->setAdvertisementData(adv);
        
        NimBLEAdvertisementData scanResp;
        scanResp.setName(getMacName().c_str());
        pAdvertising->setScanResponseData(scanResp);
        
        pAdvertising->enableScanResponse(true);
        pAdvertising->start();
        Serial.println("[ADV] Advertising Xbox Gamepad.");
    }
}

// ===================== ЯДРО 1: СВЕРХБЫСТРЫЙ ЖЕСТКИЙ ЗАХВАТ АЦП (HARD REAL-TIME) =====================
void adcAcquisitionTask(void* pvParameters) {
    while (true) {
        // Ожидаем прерывания от АЦП без процессорного поллинга (джиттер < 1 мкс)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        AdcOutput raw1 = adc1.readAdcRaw();
        AdcOutput raw2 = adc2.readAdcRaw();

        // Заполняем кольцевой буфер с защитой мьютекса
        xSemaphoreTake(bufferMutex, portMAX_DELAY);
        for (int i = 0; i < 8; i++) {
            eegRing[i][eeg_head]   = (float)raw1.ch[i].i * UV_SCALE;
            eegRing[i+8][eeg_head] = (float)raw2.ch[i].i * UV_SCALE;
        }
        eeg_head = (eeg_head + 1) & 255;
        xSemaphoreGive(bufferMutex);

        // Транслируем кастомный пакет сырых данных по BLE, если клиент подключен
        if (pDataCharacteristic && pServer && pServer->getConnectedCount() > 0) {
            customPacket[1] = sampleCounter++;

            // Заполняем каналы 1-8
            for (int i = 0; i < 8; i++) {
                int32_t v = raw1.ch[i].i;
                customPacket[2 + i*3 + 0] = (v >> 16) & 0xFF;
                customPacket[2 + i*3 + 1] = (v >> 8) & 0xFF;
                customPacket[2 + i*3 + 2] = v & 0xFF;
            }

            // Заполняем каналы 9-16
            for (int i = 0; i < 8; i++) {
                int32_t v = raw2.ch[i].i;
                customPacket[26 + i*3 + 0] = (v >> 16) & 0xFF;
                customPacket[26 + i*3 + 1] = (v >> 8) & 0xFF;
                customPacket[26 + i*3 + 2] = v & 0xFF;
            }

            pDataCharacteristic->notify(customPacket, 51);
        }

        // Будим Ядро 0 для расчета DSP строго каждые 4 семпла (16 мс)
        static uint8_t sampleCounterLocal = 0;
        if (++sampleCounterLocal >= 4) {
            sampleCounterLocal = 0;
            if (DSPTaskHandle != NULL) xTaskNotifyGive(DSPTaskHandle);
        }
    }
}

// ===================== ЯДРО 0: ИЗОЛИРОВАННЫЙ МАТЕМАТИЧЕСКИЙ РАСЧЕТ =====================
void dspCoreTask(void* pvParameters) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int head;
        int ch = activeChannels;

        xSemaphoreTake(bufferMutex, portMAX_DELAY);
        head = eeg_head;
        for (int c = 0; c < ch; c++) {
            memcpy(localEEG[c], eegRing[c], sizeof(float) * BUF_SIZE);
        }
        xSemaphoreGive(bufferMutex);

        // 1. CAR (Усреднение по каналам)
        float pInvCh = 1.0f / (float)ch;
        for (int t = 0; t < BUF_SIZE; t++) {
            int idx = (head + t) & 255;
            float avg = 0.0f;
            for (int c = 0; c < ch; c++) avg += localEEG[c][idx];
            avg *= pInvCh;
            for (int c = 0; c < ch; c++) {
                re[c][t] = localEEG[c][idx] - avg;
                im[c][t] = 0.0f;
            }
        }

        // 2. FFT & Нормализация
        for (int c = 0; c < ch; c++) {
            fast_fft(re[c], im[c]);
            re[c][51] = im[c][51] = 0.0f;
            re[c][102] = im[c][102] = 0.0f;

            for (int k = 18; k <= 36; k++) {
                float mag = sqrtf(re[c][k] * re[c][k] + im[c][k] * im[c][k]) + 1e-6f;
                float inv_mag = 1.0f / mag;
                norm_re[c][k] = re[c][k] * inv_mag;
                norm_im[c][k] = im[c][k] * inv_mag;
            }
        }

        // 3. Вычисление Топологического Тензора (ciPLV) — 1 в 1 как в Донгле
        float tvx = 0.0f, tvy = 0.0f, ttq = 0.0f;
        float div = 0.0f, energy = 0.0f;
        int pairIdx = 0;
        
        float inv_19 = 1.0f / 19.0f;

        for (int i = 0; i < ch; i++) {
            float ex_i = EX[i], ey_i = EY[i];
            for (int j = i + 1; j < ch; j++) {
                float si = 0.0f;
                for (int k = 18; k <= 36; k++) {
                    si += (norm_im[i][k] * norm_re[j][k] - norm_re[i][k] * norm_im[j][k]);
                }
                float move_val = si * inv_19; 
                
                energy += fabsf(move_val); // Общая энергия
                
                float dx = EX[j] - ex_i;
                float dy = EY[j] - ey_i;
                
                tvx += move_val * dx; 
                tvy += move_val * dy;
                ttq += (move_val * (ex_i * dy - ey_i * dx)) / 100.0f;
                div += move_val * (dx * ex_i + dy * ey_i) / 10.0f;
                
                pairIdx++;
            }
        }

        // 4. Скейлинг под количество пар
        float scale = 28.0f / (float)pairIdx;
        float target_vx = tvx * scale;
        float target_vy = tvy * scale;
        float target_tq = ttq * scale;

        // 5. Вычисление Persistence
        float mag = sqrtf(target_vx*target_vx + target_vy*target_vy) + 1e-6f;
        float cosTheta = (target_vx*lastTX + target_vy*lastTY) / (mag * sqrtf(lastTX*lastTX + lastTY*lastTY) + 1e-6f);
        if (mag > 0.05f && cosTheta > 0.8f) persistence = fminf(1.0f, persistence + 0.05f);
        else persistence *= 0.97f;
        lastTX = target_vx; lastTY = target_vy;

        // 6. Вычисление сырых значений стиков (1-в-1 как в exportAxes из App.tsx)
        float raw_lx = target_vx / 15.0f;
        float raw_ly = -target_vy / 15.0f; 
        float raw_rx = target_tq / 2.0f; // Скомпенсированный по амплитуде поворот

        // 7. СГЛАЖИВАНИЕ ОТКЛЮЧЕНО (0 мс задержки, реактивность 1-в-1 как в Python-скрипте)
        smooth_lx = raw_lx;
        smooth_ly = raw_ly;
        smooth_rx = raw_rx;

        // Ограничиваем физический диапазон стиков строго в пределах [-1.0 ... 1.0]
        smooth_lx = fmaxf(-1.0f, fminf(1.0f, smooth_lx));
        smooth_ly = fmaxf(-1.0f, fminf(1.0f, smooth_ly));
        smooth_rx = fmaxf(-1.0f, fminf(1.0f, smooth_rx));

        // --- ВЫВОД 1: ФИЗИЧЕСКИЙ USB ГЕЙМПАД ---
        int8_t usb_lx = (int8_t)(smooth_lx * 127.0f);
        int8_t usb_ly = (int8_t)(smooth_ly * 127.0f);
        int8_t usb_rx = (int8_t)(smooth_rx * 127.0f);

        Gamepad.leftStick(usb_lx, usb_ly);
        Gamepad.rightStick(usb_rx, 0); 

        // --- ВЫВОД 2: БЕСПРОВОДНОЙ BLE ГЕЙМПАД ---
        if (bleGamepad.isConnected()) {
            int16_t ble_lx = (int16_t)(smooth_lx * 32767.0f);
            int16_t ble_ly = (int16_t)(smooth_ly * 32767.0f);
            int16_t ble_rx = (int16_t)(smooth_rx * 32767.0f);

            bleGamepad.setAxes(ble_lx, ble_ly, ble_rx, 0, 0, 0, DPAD_CENTERED);

            static int heartbeat = 0;
            if (heartbeat++ > 30) { 
                bleGamepad.press(BUTTON_15);
                bleGamepad.release(BUTTON_15);
                heartbeat = 0;
            }

            bleGamepad.sendReport();
        }
    }
}

// ===================== ИНИЦИАЛИЗАЦИЯ (SETUP) =====================
void setup() {
    Serial.begin(115200);

    initDSP();
    bufferMutex = xSemaphoreCreateMutex();

    SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
    
    // Инициализация SPI и пинов для двух чипов. 
    adc1.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS1, PIN_DRDY1, PIN_RESET);
    adc2.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS2, -1, PIN_RESET);
    adc1.reset();

    // Настройка АЦП (усиление x4 по умолчанию)
    adc1.setOsr(OSR_16384); 
    adc1.writeRegisterMasked(0x08, 0x0F, 0x000F); // DC Block > 1Hz
    adc1.writeRegister(REG_GAIN1, 0x2222); 
    adc1.writeRegister(REG_GAIN2, 0x2222); 

    adc2.setOsr(OSR_16384); 
    adc2.writeRegisterMasked(0x08, 0x0F, 0x000F); // DC Block > 1Hz
    adc2.writeRegister(REG_GAIN1, 0x2222); 
    adc2.writeRegister(REG_GAIN2, 0x2222); 

    customPacket[0] = 0xA0;
    customPacket[50] = 0xC0; 

    // === ИНИЦИАЛИЗАЦИЯ USB ГЕЙМПАДА ===
    Gamepad.begin();
    USB.begin(); 
    
    // Тестовый клик при старте
    Gamepad.pressButton(0);
    delay(80);
    Gamepad.releaseButton(0);

    // === АВТООПРЕДЕЛЕНИЕ МОДЕЛИ ПЛАТЫ (FreeEEG8 / FreeEEG16) ===
    uint16_t id2 = adc2.readRegister(REG_GAIN1);
    if (id2 == 0x0000 || id2 == 0xFFFF) {
        activeChannels = 8;
        Serial.println("[SYSTEM] ADC2 not found. Operating in 8-channel mode (FreeEEG8).");
    } else {
        activeChannels = 16;
        Serial.println("[SYSTEM] ADC2 found. Operating in 16-channel mode (FreeEEG16).");
    }

    // Заполняем пространственные координаты электродов в строгом соответствии с топологией
    if (activeChannels == 8) {
        for (int i = 0; i < 8; i++) {
            float rad = ANGLES_8[i] * PI / 180.0f;
            EX[i] = cosf(rad) * 10.0f; // RADIUS = 10.0f, как в BleService.ts
            EY[i] = sinf(rad) * 10.0f;
        }
        Serial.println("[DSP] 8-channel radial electrodes map initialized.");
    } else {
        for (int i = 0; i < 16; i++) {
            EX[i] = COORDS_16_X[i];
            EY[i] = COORDS_16_Y[i];
        }
        Serial.println("[DSP] 16-channel concentric electrodes map initialized.");
    }

    // === ШАГ 1: ПЕРВИЧНАЯ ИНИЦИАЛИЗАЦИЯ СТЕКА BLE И НАСТРОЙКА MTU (БЕЗОПАСНЫЙ СТАРТ) ===
    NimBLEDevice::init(getMacName().c_str());
    NimBLEDevice::setPower(ESP_PWR_LVL_P3);
    NimBLEDevice::setMTU(256); // Резервируем MTU для 16 каналов

    // === ШАГ 2: СОЗДАЕМ СЕРВЕР И РЕГИСТРИРУЕМ СТРУКТУРУ КАСТОМНОГО СЕРВИСА ===
    pServer = NimBLEDevice::createServer();
    NimBLEService* pService = nullptr;
    if (pServer) {
        pService = pServer->createService(SERVICE_UUID);
        pDataCharacteristic = pService->createCharacteristic(DATA_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
        pCmdCharacteristic = pService->createCharacteristic(CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
        pCmdCharacteristic->setCallbacks(new CmdCallbacks());
    }

    // === ШАГ 3: ИНИЦИАЛИЗИРУЕМ БИБЛИОТЕКУ ГЕЙМПАДА ===
    bleGamepadConfig.setVid(0x045E); // Microsoft
    bleGamepadConfig.setPid(0x02FD); // Xbox Wireless Controller BLE
    bleGamepadConfig.setAutoReport(false); 
    bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
    bleGamepadConfig.setButtonCount(16);
    bleGamepadConfig.setHatSwitchCount(1);
    
    // Явно разрешаем использование всех осей в библиотеке BleGamepad
    bleGamepadConfig.setWhichAxes(true, true, true, true, true, true, true, true);
    
    // Задаем BLE-осям полный 16-битный диапазон для максимальной точности
    bleGamepadConfig.setAxesMin(-32767); 
    bleGamepadConfig.setAxesMax(32767);

    bleGamepad.begin(&bleGamepadConfig);
    Serial.println("Gamepad Service Integrated.");

    // === ШАГ 4: ТЕПЕРЬ ЗАПУСКАЕМ КАСТОМНЫЙ ЭЭГ-СЕРВИС ===
    if (pService) {
        pService->start();
    }

    // === КРИТИЧЕСКАЯ ЗАДЕРЖКА ДЛЯ СТАБИЛИЗАЦИИ ===
    delay(500);

    // === ШАГ 5: БЕЗОПАСНАЯ ПЕРЕНАСТРОЙКА РЕКЛАМЫ ===
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    if (pAdvertising) {
        pAdvertising->stop();
        delay(100); // пауза после остановки

        // Основной рекламный пакет (Primary Advertisement)
        NimBLEAdvertisementData primaryAdv;
        primaryAdv.setFlags(0x06); 
        primaryAdv.setCompleteServices(NimBLEUUID(SERVICE_UUID)); // Только ЭЭГ служба
        pAdvertising->setAdvertisementData(primaryAdv);

        // Пакет скан-ответа (Scan Response)
        NimBLEAdvertisementData scanResponse;
        scanResponse.setName(getMacName().c_str());
        scanResponse.setCompleteServices(NimBLEUUID("1812")); // HID служба для геймпада
        pAdvertising->setScanResponseData(scanResponse);

        pAdvertising->enableScanResponse(true);
        pAdvertising->start();
        Serial.println("Combined Advertising configured and active.");
    }

    // === СОЗДАНИЕ ВЫДЕЛЕННЫХ ЗАДАЧ НА РАЗНЫХ ЯДРАХ ===
    
    // 1. Задача захвата АЦП на Ядре 1 с высоким приоритетом 10 (HARD REAL-TIME)
    xTaskCreatePinnedToCore(adcAcquisitionTask, "ADCTask", 8192, NULL, 10, &ADCTaskHandle, 1);
    
    // 2. Задача расчета DSP на Ядре 0 с приоритетом 3
    xTaskCreatePinnedToCore(dspCoreTask, "DSPTask", 10240, NULL, 3, &DSPTaskHandle, 0);

    pinMode(PIN_DRDY1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY1), onDrdy, FALLING);
}

// ===================== ЯДРО 1: НИЗКОПРИОРИТЕТНЫЙ СИСТЕМНЫЙ ЦИКЛ =====================
void loop() {
    // loop() освобожден от поллинга DRDY и работает на самом низком приоритете (1).
    // Медленные операции больше не создают фазовый шум для АЦП.

    // --- ПЕРЕХВАТ И АВТОМАТИЧЕСКИЙ ЗАПРОС ПАРАМЕТРОВ СВЯЗИ ДЛЯ ANDROID ---
    static bool wasConnected = false;
    if (pServer && pServer->getConnectedCount() > 0) {
        if (!wasConnected) {
            wasConnected = true;
            Serial.println("[SYSTEM] Client connected! Forcing fast BLE connection parameters (MTU fix)...");
            
            std::vector<uint16_t> connHandles = pServer->getPeerDevices();
            for (uint16_t handle : connHandles) {
                pServer->updateConnParams(handle, 6, 6, 0, 100); 
            }
        }
    } else {
        wasConnected = false;
    }

    // --- ДИНАМИЧЕСКАЯ РОТАЦИЯ РЕКЛАМЫ (TIMESHARING) ЧЕРЕЗ КАЖДЫЕ 5 СЕКУНД ---
    static unsigned long lastAdvSwitch = millis();
    static bool advState = false; // false = ЭЭГ, true = Геймпад
    
    // Меняем тип рекламы только если в данный момент никто не подключен
    if (pServer && pServer->getConnectedCount() == 0) {
        if (millis() - lastAdvSwitch > 5000) {
            lastAdvSwitch = millis();
            advState = !advState;
            if (advState) {
                startAdvertisingGamepad();
            } else {
                startAdvertisingEEG();
            }
        }
    }

    // --- ОБРАБОТКА ЧТЕНИЯ РЕГИСТРА ЧЕРЕЗ ПРОЗРАЧНЫЙ МОСТ ---
    if (has_pending_read) {
        uint16_t val = adc1.readRegister(reg_addr);
        uint8_t tx[3] = { reg_addr, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
        pCmdCharacteristic->notify(tx, 3);
        Serial.printf("Read Reg 0x%02X -> 0x%04X\n", reg_addr, val);
        has_pending_read = false;
    }

    // --- ОБРАБОТКА ЗАПИСИ ---
    if (has_pending_write) {
        if (use_mask) {
            adc1.writeRegisterMasked(reg_addr, reg_val, reg_mask);
            adc2.writeRegisterMasked(reg_addr, reg_val, reg_mask);
            Serial.printf("Write Masked: 0x%02X, Val 0x%04X, Mask 0x%04X\n", reg_addr, reg_val, reg_mask);
        } else {
            adc1.writeRegister(reg_addr, reg_val);
            adc2.writeRegister(reg_addr, reg_val);
            Serial.printf("Write Direct: 0x%02X, Val 0x%04X\n", reg_addr, reg_val);
        }
        has_pending_write = false;
    }

    // Разгружаем ядро для выполнения внутренних фоновых процессов FreeRTOS/Arduino
    delay(10);
}
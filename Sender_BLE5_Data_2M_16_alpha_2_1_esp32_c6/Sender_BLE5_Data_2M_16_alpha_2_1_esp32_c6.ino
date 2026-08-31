#include <Arduino.h>
#include "esp_mac.h"
#include <SPI.h>
#include "ADS131M08.h"
#include <NimBLEDevice.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DATA_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CMD_CHAR_UUID       "c0de0001-36e1-4688-b7f5-ea07361b26a8" // Характеристика прозрачного моста

// Пины
#define PIN_CS1    D0  
#define PIN_DRDY1  D1  // DRDY берем только с первого АЦП
#define PIN_RESET  D2  // Общий сброс для обоих чипов
#define PIN_CS2    D3  
#define PIN_SCLK   D8  
#define PIN_MISO   D9  
#define PIN_MOSI   D10 
#define PIN_CLKOUT D6

// Создаем два объекта для двух АЦП
ADS131M08 adc1;
ADS131M08 adc2;

NimBLECharacteristic* pDataCharacteristic = nullptr;
NimBLECharacteristic* pCmdCharacteristic = nullptr;

volatile bool deviceConnected = false;
volatile bool drdyTriggered = false;

// Единый кастомный 16-канальный пакет: 
// 1(Header) + 1(Counter) + 24(ADC1) + 24(ADC2) + 1(Footer) = 51 байт
uint8_t customPacket[51];
uint8_t sampleCounter = 0;

// === ФЛАГИ ПРОЗРАЧНОГО МОСТА ===
volatile bool has_pending_read = false;
volatile bool has_pending_write = false;
volatile uint8_t  reg_addr = 0;
volatile uint16_t reg_val = 0;
volatile uint16_t reg_mask = 0;
volatile bool use_mask = false;
volatile bool needs_adv_restart = false;

void IRAM_ATTR onDrdy() { drdyTriggered = true; }

// === КОЛЛБЕКИ КОМАНД С МОБИЛКИ ===
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

// Переменные для отложенного согласования параметров связи
volatile uint16_t activeConnHandle = 0;
volatile unsigned long connectTime = 0;
volatile bool paramUpdatePending = false;

class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        deviceConnected = true;
        activeConnHandle = connInfo.getConnHandle();
        connectTime = millis();
        paramUpdatePending = true; // Выставляем флаг для отложенного запроса в loop
        Serial.println("BLE Connected! Waiting 2 seconds before parameter negotiation...");
    }
    
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        deviceConnected = false;
        paramUpdatePending = false;
        needs_adv_restart = true; 
        Serial.printf("BLE Disconnected. Reason: %d\n", reason);
    }
};

void setup() {
    Serial.begin(115200);

    SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);

    adc1.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS1, PIN_DRDY1, PIN_RESET);
    adc2.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS2, -1, PIN_RESET);
    
    adc1.reset();

    adc1.setOsr(OSR_16384); 
    adc1.writeRegisterMasked(0x08, 0x0F, 0x000F);
    adc1.writeRegister(REG_GAIN1, 0x2222);
    adc1.writeRegister(REG_GAIN2, 0x2222); 

    adc2.setOsr(OSR_16384); 
    adc2.writeRegisterMasked(0x08, 0x0F, 0x000F);
    adc2.writeRegister(REG_GAIN1, 0x2222);
    adc2.writeRegister(REG_GAIN2, 0x2222); 

    customPacket[0] = 0xA0;
    customPacket[50] = 0xC0; 

    // BLE Инициализация
    NimBLEDevice::init("FreeEEG16");
    
    // Повышаем мощность передатчика для устойчивости к помехам и коллизиям
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); // +9 dBm (значительно улучшает SNR)
    
    // Включаем поддержку MTU до 64 байт для предотвращения фрагментации 51-байтового пакета
    NimBLEDevice::setMTU(64); 

    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    NimBLEService *pService = pServer->createService(SERVICE_UUID);
    
    pDataCharacteristic = pService->createCharacteristic(DATA_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    
    pCmdCharacteristic = pService->createCharacteristic(CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    pCmdCharacteristic->setCallbacks(new CmdCallbacks());

    pService->start();

    NimBLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
    NimBLEDevice::getAdvertising()->start();

    pinMode(PIN_DRDY1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_DRDY1), onDrdy, FALLING);
}

void loop() {
    // --- ПЕРЕЗАПУСК РЕКЛАМЫ (АВТО-РЕКОННЕКТ) ---
    if (needs_adv_restart) {
        delay(500); 
        NimBLEDevice::startAdvertising();
        needs_adv_restart = false;
        Serial.println("Advertising restarted. Ready for reconnect.");
    }

    // --- ОТЛОЖЕННЫЙ ЗАПРОС ПАРАМЕТРОВ СВЯЗИ ---
    // Договариваемся о таймслотах строго через 2 секунды после подключения,
    // когда GATT-процедуры хоста полностью завершены.
    if (deviceConnected && paramUpdatePending && (millis() - connectTime > 2000)) {
        paramUpdatePending = false;
        
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_BT);
        
        // Разносим фазы связи устройств на основе их MAC-адресов
        int offset = (mac[5] % 4) * 2; 
        uint16_t min_int = 16 + offset; // Интервал от 20 мс до 27.5 мс
        uint16_t max_int = min_int + 4;  // Небольшой люфт для планировщика ОС
        
        NimBLEDevice::getServer()->updateConnParams(activeConnHandle, min_int, max_int, 0, 400);
        Serial.printf("[SYSTEM] Connection parameters negotiated: min=%d, max=%d\n", min_int, max_int);
    }

    // 1. ОБРАБОТКА ЧТЕНИЯ РЕГИСТРА
    if (has_pending_read) {
        uint16_t val = adc1.readRegister(reg_addr);
        uint8_t tx[3] = { reg_addr, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
        pCmdCharacteristic->notify(tx, 3);
        has_pending_read = false;
    }

    // 2. ОБРАБОТКА ЗАПИСИ В РЕГИСТРЫ
    if (has_pending_write) {
        if (use_mask) {
            adc1.writeRegisterMasked(reg_addr, reg_val, reg_mask);
            adc2.writeRegisterMasked(reg_addr, reg_val, reg_mask);
        } else {
            adc1.writeRegister(reg_addr, reg_val);
            adc2.writeRegister(reg_addr, reg_val);
        }
        has_pending_write = false;
    }

    // 3. ОТПРАВКА ДАННЫХ АЦП (16 КАНАЛОВ)
    if (drdyTriggered) {
        drdyTriggered = false;
        
        AdcOutput raw1 = adc1.readAdcRaw();
        AdcOutput raw2 = adc2.readAdcRaw();

        if (deviceConnected) {
            customPacket[1] = sampleCounter++;

            for (int i = 0; i < 8; i++) {
                int32_t v = raw1.ch[i].i;
                customPacket[2 + i*3 + 0] = (v >> 16) & 0xFF;
                customPacket[2 + i*3 + 1] = (v >> 8) & 0xFF;
                customPacket[2 + i*3 + 2] = v & 0xFF;
            }

            for (int i = 0; i < 8; i++) {
                int32_t v = raw2.ch[i].i;
                customPacket[26 + i*3 + 0] = (v >> 16) & 0xFF;
                customPacket[26 + i*3 + 1] = (v >> 8) & 0xFF;
                customPacket[26 + i*3 + 2] = v & 0xFF;
            }

            // Пакет будет отправлен цельным кадром за один раз
            pDataCharacteristic->notify(customPacket, 51);
            
        } else if (!sampleCounter){
            needs_adv_restart = true; 
        }
    }
}

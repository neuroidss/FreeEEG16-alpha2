#include <Arduino.h>
#include <NimBLEDevice.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DATA_CHAR_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CMD_CHAR_UUID       "c0de0001-36e1-4688-b7f5-ea07361b26a8" // Командный мост
#define MAX_CONNECTIONS     8  // Поднято до 8 для поддержки 7 девайсов

// Математически безупречный расширенный пул взаимно простых чисел (18.75мс - 53.75мс)
const uint16_t coprimePool[] = {15, 16, 17, 19, 23, 29, 31, 37, 41, 43}; 
#define COPRIME_POOL_SIZE 10

typedef struct {
    NimBLEAddress address;
    NimBLEClient* pClient;             
    NimBLERemoteCharacteristic* pCmdChar; // Характеристика для отправки команд тюнинга
    volatile uint32_t packetsReceived; 
    volatile uint32_t totalReceived;   
    volatile uint32_t totalLost;       
    volatile uint8_t lastCounter;
    volatile bool active;
    
    // Переменные автотюнера
    uint16_t baseInterval;             
    uint16_t currentInterval;          
    bool globalChopEnabled;            // Статус Global-Chop (true: 83Hz, false: 250Hz)
    uint8_t tuneAttempts;              // Кол-во попыток мягкого сдвига интервала
    uint32_t lastTuneTime;             // Время последнего изменения параметров
} DeviceStats;

typedef struct __attribute__((packed)) tune_request {
    NimBLEAddress address;
    uint16_t interval;
} tune_request;

DeviceStats devices[MAX_CONNECTIONS];
QueueHandle_t connectQueue;
QueueHandle_t deleteQueue; 
QueueHandle_t tuneQueue; 
SemaphoreHandle_t statsMutex;

volatile bool isConnecting = false;

// Состояние ИИ-агента
volatile int agentState = 0; // 0: Ожидание/Свободен, 1: Стабилизация (Settle Phase)
volatile int settleTimer = 0;

uint32_t lastConnectionTime = 0;
uint32_t lastTickTime = 0;
uint32_t previousLost[MAX_CONNECTIONS]; // Хранит исторические потери по каждому каналу
bool scanningActive = true;

// Отправка команды на плату (запись маскированного регистра REG_CFG [0x06])
void setDeviceGlobalChop(NimBLERemoteCharacteristic* pCmdChar, bool enable) {
    if (pCmdChar == nullptr) return;
    uint8_t payload[5];
    payload[0] = 0x06;                     // Регистр REG_CFG
    payload[1] = enable ? 0x01 : 0x00;     // Значение MSB (бит 8: GC_EN = 1 или 0)
    payload[2] = 0x00;                     // Значение LSB
    payload[3] = 0x01;                     // Маска MSB (0x0100 - маскируем только бит 8)
    payload[4] = 0x00;                     // Маска LSB
    pCmdChar->writeValue(payload, 5, false); // Без подтверждения для скорости
}

void notifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic,
                    uint8_t* pData, size_t length, bool isNotify) {
    if (length == 51 && pData[0] == 0xA0 && pData[50] == 0xC0) {
        
        const NimBLERemoteService* pSvc = pBLERemoteCharacteristic->getRemoteService();
        if (pSvc == nullptr) return;
        
        const NimBLEClient* pClient = pSvc->getClient();
        if (pClient == nullptr) return;
        
        NimBLEAddress addr = pClient->getPeerAddress();
        uint8_t counter = pData[1];
        
        if (xSemaphoreTake(statsMutex, 0) == pdTRUE) {
            for (int i = 0; i < MAX_CONNECTIONS; i++) {
                if (devices[i].active && devices[i].address == addr) {
                    devices[i].packetsReceived++;
                    devices[i].totalReceived++;
                    
                    if (devices[i].lastCounter != 255) {
                        uint8_t expected = (devices[i].lastCounter + 1) % 256;
                        if (counter != expected) {
                            uint8_t lost = (counter - expected) % 256;
                            devices[i].totalLost += lost;
                        }
                    }
                    devices[i].lastCounter = counter;
                    break;
                }
            }
            xSemaphoreGive(statsMutex);
        }
    }
}

class ClientCallbacks : public NimBLEClientCallbacks {
    void onDisconnect(NimBLEClient* pClient, int reason) override {
        NimBLEAddress addr = pClient->getPeerAddress();
        Serial.printf("\n[DONGLE] Disconnected from %s, reason: %d\n", addr.toString().c_str(), reason);
        
        if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < MAX_CONNECTIONS; i++) {
                if (devices[i].active && devices[i].address == addr) {
                    devices[i].active = false;
                    break;
                }
            }
            isConnecting = false;
            lastConnectionTime = millis();
            xSemaphoreGive(statsMutex);
        }
        
        xQueueSend(deleteQueue, &pClient, 0);
        
        if (!scanningActive) {
            NimBLEDevice::getScan()->start(0, false);
            scanningActive = true;
            Serial.println("[DONGLE] Scanning RESTARTED to find lost device.");
        }
    }

    bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) override {
        return false; 
    }
};

class MyScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
        if (advertisedDevice->isAdvertisingService(NimBLEUUID(SERVICE_UUID))) {
            NimBLEAddress addr = advertisedDevice->getAddress();
            bool alreadyConnected = false;
            bool shouldQueue = false;

            if (xSemaphoreTake(statsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                for (int i = 0; i < MAX_CONNECTIONS; i++) {
                    if (devices[i].active && devices[i].address == addr) {
                        alreadyConnected = true;
                        break;
                    }
                }
                
                int activeCount = 0;
                for (int i = 0; i < MAX_CONNECTIONS; i++) {
                    if (devices[i].active) activeCount++;
                }

                if (!alreadyConnected && !isConnecting && (activeCount < MAX_CONNECTIONS)) {
                    isConnecting = true;
                    shouldQueue = true;
                }
                xSemaphoreGive(statsMutex);
            }
            if (shouldQueue) {
                xQueueSend(connectQueue, &addr, 0);
            }
        }
    }
};

void connectToAddress(NimBLEAddress addr) {
    NimBLEClient* pClient = NimBLEDevice::createClient();
    if (pClient == nullptr) {
        Serial.println("[DONGLE] Error: Failed to create BLE client!");
        if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
            isConnecting = false;
            xSemaphoreGive(statsMutex);
        }
        return;
    }
    pClient->setClientCallbacks(new ClientCallbacks(), true); 
    
    if (pClient->connect(addr)) {
        pClient->setDataLen(251);
        pClient->updatePhy(2, 2); 
        pClient->exchangeMTU();
        delay(500); 
        
        int slotIndex = 0;
        if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < MAX_CONNECTIONS; i++) {
                if (!devices[i].active) {
                    slotIndex = i;
                    break;
                }
            }
            xSemaphoreGive(statsMutex);
        }
        
        uint16_t interval = coprimePool[slotIndex];
        pClient->updateConnParams(interval, interval, 0, 200); 
        delay(500); 
        
        const NimBLERemoteService* pSvc = pClient->getService(SERVICE_UUID);
        if (pSvc) {
            NimBLERemoteCharacteristic* pChar = pSvc->getCharacteristic(DATA_CHAR_UUID);
            NimBLERemoteCharacteristic* pCmdChar = pSvc->getCharacteristic(CMD_CHAR_UUID);
            
            if (pChar && pChar->canNotify()) {
                if (pChar->subscribe(true, notifyCallback)) {
                    
                    // Гарантированно отключаем ЧОП на плате при старте, чтобы сбросить её в 250 Гц!
                    if (pCmdChar) {
                        setDeviceGlobalChop(pCmdChar, false);
                        delay(100); 
                    }
                    
                    if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
                        for (int i = 0; i < MAX_CONNECTIONS; i++) {
                            if (!devices[i].active) {
                                devices[i].address = addr;
                                devices[i].pClient = pClient; 
                                devices[i].pCmdChar = pCmdChar;
                                devices[i].packetsReceived = 0;
                                devices[i].totalReceived = 0;
                                devices[i].totalLost = 0;
                                devices[i].lastCounter = 255;
                                devices[i].baseInterval = interval;
                                devices[i].currentInterval = interval;
                                devices[i].globalChopEnabled = false; 
                                devices[i].tuneAttempts = 0;
                                devices[i].lastTuneTime = millis();
                                devices[i].active = true;
                                
                                previousLost[i] = 0; // Сбрасываем историю потерь для нового подключения
                                break;
                            }
                        }
                        lastConnectionTime = millis(); 
                        isConnecting = false; 
                        xSemaphoreGive(statsMutex);
                    }
                    return;
                }
            }
        }
        pClient->disconnect(); 
    } else {
        NimBLEDevice::deleteClient(pClient); 
    }
}

void sendTuneRequest(NimBLEAddress addr, uint16_t interval) {
    tune_request req;
    req.address = addr;
    req.interval = interval;
    xQueueSend(tuneQueue, &req, 0);
}

void connectTask(void *pvParameters) {
    NimBLEAddress addr;
    NimBLEClient* clientToDelete;
    tune_request tuneReq;

    while(1) {
        while (xQueueReceive(deleteQueue, &clientToDelete, 0)) {
            NimBLEDevice::deleteClient(clientToDelete);
        }

        while (xQueueReceive(tuneQueue, &tuneReq, 0)) {
            if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < MAX_CONNECTIONS; i++) {
                    if (devices[i].active && devices[i].address == tuneReq.address) {
                        if (devices[i].pClient && devices[i].pClient->isConnected()) {
                            devices[i].pClient->updateConnParams(tuneReq.interval, tuneReq.interval, 0, 200);
                        }
                        break;
                    }
                }
                xSemaphoreGive(statsMutex);
            }
            delay(150); 
        }

        if (xQueueReceive(connectQueue, &addr, pdMS_TO_TICKS(100))) {
            NimBLEDevice::getScan()->stop(); 
            connectToAddress(addr);
            
            int activeCount = 0;
            if (xSemaphoreTake(statsMutex, portMAX_DELAY) == pdTRUE) {
                for (int i = 0; i < MAX_CONNECTIONS; i++) {
                    if (devices[i].active) activeCount++;
                }
                isConnecting = false; 
                xSemaphoreGive(statsMutex);
            }
            
            if (activeCount < MAX_CONNECTIONS && scanningActive) {
                NimBLEDevice::getScan()->start(0, false); 
            }
        }
    }
}

void setup() {
    Serial.begin(1500000); // 1.5 Мбит/с

    statsMutex = xSemaphoreCreateMutex();
    connectQueue = xQueueCreate(10, sizeof(NimBLEAddress));
    deleteQueue = xQueueCreate(10, sizeof(NimBLEClient*)); 
    tuneQueue = xQueueCreate(10, sizeof(tune_request)); 

    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        devices[i].active = false;
        previousLost[i] = 0;
    }

    NimBLEDevice::init("FreeEEG_Diag_Dongle");
    NimBLEDevice::setMTU(64); 

    xTaskCreatePinnedToCore(connectTask, "connectTask", 4096, NULL, 1, NULL, 0);

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(new MyScanCallbacks()); 
    pScan->setActiveScan(true); // Активный скан
    
    pScan->setInterval(400); 
    pScan->setWindow(40);   
    
    pScan->start(0, false); 
    lastConnectionTime = millis();
    lastTickTime = millis();
}

void loop() {
    delay(1000);
    
    DeviceStats localDevices[MAX_CONNECTIONS];
    int localConnectedCount = 0;
    bool statsCopied = false;
    
    uint32_t now = millis();
    uint32_t elapsedMs = now - lastTickTime;
    lastTickTime = now;
    
    if (xSemaphoreTake(statsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(localDevices, devices, sizeof(devices));
        
        // Динамический подсчет активных плат (исключает баг ухода в отрицательные значения)
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (devices[i].active) {
                localConnectedCount++;
            }
        }
        
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            devices[i].packetsReceived = 0;
        }
        statsCopied = true;
        xSemaphoreGive(statsMutex);
    }
    
    // 2. СИСТЕМНЫЙ ДВУХСТУПЕНЧАТЫЙ ИИ-АГЕНТ ОБРАТНОЙ СВЯЗИ (Параллельный опрос всех каналов)
    if (statsCopied) {
        if (agentState == 1) { // Режим ожидания стабилизации (Settle Phase)
            settleTimer--;
            if (settleTimer == 0) {
                agentState = 0; 
            }
        } 
        else if (agentState == 0) {
            bool performedTuning = false;
            
            // Локальная карта занятых интервалов, чтобы не назначить один копрайм двум платам в одну секунду
            bool intervalUsed[60] = {false}; // 60 достаточно для расширенного пула
            for (int d = 0; d < MAX_CONNECTIONS; d++) {
                if (localDevices[d].active) {
                    intervalUsed[localDevices[d].currentInterval] = true;
                }
            }
            
            // Пробегаем по ВСЕМ каналам параллельно за одну секунду!
            for (int i = 0; i < MAX_CONNECTIONS; i++) {
                if (localDevices[i].active) {
                    uint32_t lostThisSecond = localDevices[i].totalLost - previousLost[i];
                    
                    // Если зафиксирована реальная потеря пакетов на этом канале
                    if (lostThisSecond > 0) {
                        
                        // ЕСЛИ ПЛАТА УЖЕ НА ЧОПЕ — БОЛЬШЕ ЕЁ НЕ ТРОГАЕМ («править бесполезно»).
                        // Всю вычислительную мощность отдаем на спасение плат, которые ещё на 250 Гц!
                        if (localDevices[i].globalChopEnabled) {
                            continue; 
                        }
                        
                        // === СТУПЕНЬ 2: Перевод в Global-Chop (если мягкий тюнинг интервалов не помог за 3 попытки) ===
                        if (localDevices[i].tuneAttempts >= 3) {
                            Serial.printf("\n[AGENT] CH%d coprime tuning failed %d times. Activating Global-Chop (83Hz) to stabilize link!\n", 
                                          i + 1, localDevices[i].tuneAttempts);
                            
                            setDeviceGlobalChop(localDevices[i].pCmdChar, true); // Включаем Global-Chop
                            
                            if (xSemaphoreTake(statsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                devices[i].globalChopEnabled = true;
                                devices[i].tuneAttempts = 0;
                                devices[i].lastTuneTime = millis();
                                xSemaphoreGive(statsMutex);
                            }
                            performedTuning = true;
                        } 
                        // === СТУПЕНЬ 1: Мягкий тюнинг интервалов (копраймы и фазовые сдвиги) ===
                        else {
                            if (xSemaphoreTake(statsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                devices[i].tuneAttempts++;
                                xSemaphoreGive(statsMutex);
                            }
                            
                            // Ищем незанятый копрайм-интервал в пуле
                            uint16_t freePrimeInterval = 0;
                            for (int p = 0; p < COPRIME_POOL_SIZE; p++) {
                                uint16_t prime = coprimePool[p];
                                if (!intervalUsed[prime]) {
                                    freePrimeInterval = prime;
                                    break;
                                }
                            }
                            
                            uint16_t newInterval = 0;
                            if (freePrimeInterval != 0) {
                                newInterval = freePrimeInterval;
                                Serial.printf("\n[AGENT] CH%d lost %u pkts. Attempt %d: Moving to unused coprime interval %d (%.2f ms)\n", 
                                              i + 1, lostThisSecond, 
                                              localDevices[i].tuneAttempts, 
                                              newInterval, newInterval * 1.25);
                            } else {
                                // Сдвиг фазы на +/- 1.25 мс
                                int8_t shift = (millis() % 2 == 0) ? 1 : -1;
                                newInterval = localDevices[i].currentInterval + shift;
                                if (newInterval < 15) newInterval = 15;
                                if (newInterval > 43) newInterval = 43;
                                Serial.printf("\n[AGENT] CH%d lost %u pkts. Attempt %d: Coprimes full. Forcing phase shift to %d (%.2f ms)\n", 
                                              i + 1, lostThisSecond, 
                                              localDevices[i].tuneAttempts, 
                                              newInterval, newInterval * 1.25);
                            }
                            
                            intervalUsed[newInterval] = true; // Помечаем интервал занятым
                            
                            if (xSemaphoreTake(statsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                devices[i].currentInterval = newInterval;
                                devices[i].lastTuneTime = millis();
                                xSemaphoreGive(statsMutex);
                            }
                            
                            sendTuneRequest(localDevices[i].address, newInterval);
                            performedTuning = true;
                        }
                    }
                }
            }
            
            if (performedTuning) {
                agentState = 1;
                settleTimer = 4; // Единая фаза стабилизации на 4 сек для всей сети после параллельных обновлений
            } 
            else {
                // === ОБРАТНЫЙ КВЕСТ: Восстановление полной частоты дискретизации при длительной стабильности ===
                for (int i = 0; i < MAX_CONNECTIONS; i++) {
                    if (localDevices[i].active && localDevices[i].globalChopEnabled) {
                        uint32_t timeSinceLastTune = millis() - localDevices[i].lastTuneTime;
                        // Если активных плат мало (<= 3) и за последние 25 секунд потерь на канале нет вообще
                        if (localConnectedCount <= 3 && timeSinceLastTune > 25000) {
                            Serial.printf("\n[AGENT] Link is stable. CH%d has 0 losses. Disabling Global-Chop to restore full sample rate!\n", i + 1);
                            
                            setDeviceGlobalChop(localDevices[i].pCmdChar, false); // Отключаем Global-Chop
                            
                            if (xSemaphoreTake(statsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                devices[i].globalChopEnabled = false;
                                devices[i].tuneAttempts = 0;
                                devices[i].lastTuneTime = millis();
                                xSemaphoreGive(statsMutex);
                            }
                            
                            agentState = 1;
                            settleTimer = 5;
                            break; 
                        }
                    }
                }
            }
        }
        
        // Фиксируем текущие потери для следующего тика
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (localDevices[i].active) {
                previousLost[i] = localDevices[i].totalLost;
            }
        }
    }

    // 3. Вывод компактной статистики в одну строку (PPS вычисляется по точному времени)
    if (statsCopied) {
        if (localConnectedCount == 0) {
            Serial.printf("[DONGLE] Searching for devices... Active: 0/%d\n", MAX_CONNECTIONS);
        } else {
            Serial.printf("[DONGLE] Active: %d/%d | ", localConnectedCount, MAX_CONNECTIONS);
            for (int i = 0; i < MAX_CONNECTIONS; i++) {
                if (localDevices[i].active) {
                    const char* fullAddr = localDevices[i].address.toString().c_str();
                    const char* shortAddr = (strlen(fullAddr) >= 17) ? (fullAddr + 12) : fullAddr;
                    uint32_t normalizedPps = (localDevices[i].packetsReceived * 1000) / elapsedMs;
                    
                    Serial.printf("CH%d (%s): %u pps (%s), Lost: %u | ", 
                                  i + 1, 
                                  shortAddr, 
                                  normalizedPps, 
                                  localDevices[i].globalChopEnabled ? "GC: ON" : "GC: OFF",
                                  localDevices[i].totalLost);
                }
            }
            Serial.println();
        }
    }
    
    // 4. Отключение сканирования при активном стриме
    if (scanningActive && localConnectedCount > 0 && (millis() - lastConnectionTime > 15000)) {
        NimBLEDevice::getScan()->stop();
        scanningActive = false;
        Serial.println("\n[DONGLE] Scanner OFF for maximum radio performance.");
    }
}
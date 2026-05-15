#include <Arduino.h>
#include <SPI.h>

#define PIN_CS1    D0  
#define PIN_CS2    D3  
#define PIN_DRDY1  D1  
#define PIN_RESET  D2  
#define PIN_SCLK   D8  
#define PIN_MISO   D9  
#define PIN_MOSI   D10 

// Функция для ручного чтения регистра ID
uint16_t readChipID(uint8_t csPin, const char* chipName) {
    uint32_t cmd = 0xA00000; 

    // Отправляем команду чтения
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(csPin, LOW);
    delayMicroseconds(1);
    
    SPI.transfer((cmd >> 16) & 0xFF);
    SPI.transfer((cmd >> 8) & 0xFF);
    SPI.transfer(cmd & 0xFF);
    
    for(int i=0; i<9; i++) {
        SPI.transfer(0); SPI.transfer(0); SPI.transfer(0);
    }
    
    delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();

    delay(2); 

    // Читаем ответ
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
    digitalWrite(csPin, LOW);
    delayMicroseconds(1);
    
    uint8_t b1 = SPI.transfer(0);
    uint8_t b2 = SPI.transfer(0);
    uint8_t b3 = SPI.transfer(0);
    
    for(int i=0; i<9; i++) {
        SPI.transfer(0); SPI.transfer(0); SPI.transfer(0);
    }
    
    delayMicroseconds(1);
    digitalWrite(csPin, HIGH);
    SPI.endTransaction();

    uint16_t id_reg = (b1 << 8) | b2;
    
    Serial.printf("%s -> RAW: 0x%02X 0x%02X 0x%02X | ID: 0x%04X ", chipName, b1, b2, b3, id_reg);
    
    if (id_reg == 0x0000 || id_reg == 0xFFFF) {
        Serial.println(">>> [ERROR] DEAD OR NO RESPONSE");
    } else {
        Serial.println(">>> [SUCCESS] ALIVE!");
    }
    return id_reg;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n--- ADS131M08 DUAL HARDWARE TEST ---");

    // Настраиваем пины CS так, чтобы оба чипа молчали по умолчанию
    pinMode(PIN_CS1, OUTPUT);
    digitalWrite(PIN_CS1, HIGH);
    
    pinMode(PIN_CS2, OUTPUT);
    digitalWrite(PIN_CS2, HIGH); 

    pinMode(PIN_RESET, OUTPUT);
    
    // Аппаратный сброс обоих чипов
    digitalWrite(PIN_RESET, LOW);
    delay(10);
    digitalWrite(PIN_RESET, HIGH);
    delay(100);

    // Запускаем SPI один раз для обоих чипов
    SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
    
    // Тестируем первый чип
    readChipID(PIN_CS1, "CHIP 1 (CS1: D0)");
    
    // Тестируем второй чип
    readChipID(PIN_CS2, "CHIP 2 (CS2: D3)");
}

void loop() {
    delay(1000);
}
#include <HardwareSerial.h>

#define GPS_RX 16
#define GPS_TX 17

HardwareSerial GPSSerial(1);

// UBlox packet detection
const uint8_t UBLOX_SYNC1 = 0xB5;  // μ
const uint8_t UBLOX_SYNC2 = 0x62;  // b

// Buffer for collecting data
const int BUFFER_SIZE = 1024;
uint8_t buffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("ESP32 GPS Protocol Detector");
    Serial.println("Testing for NMEA and UBlox protocols...");
    
    // Common baud rates for GPS modules
    long baudRates[] = {9600, 38400, 115200, 57600};
    
    for (int i = 0; i < 4; i++) {
        Serial.printf("\nTesting %ld baud...\n", baudRates[i]);
        
        GPSSerial.begin(baudRates[i], SERIAL_8N1, GPS_RX, GPS_TX);
        delay(1000);
        
        unsigned long startTime = millis();
        bool foundNMEA = false;
        bool foundUBlox = false;
        int ubxSync1Count = 0;
        int ubxSync2Count = 0;
        int dollarCount = 0;
        
        // Test for 5 seconds at each baud rate
        while (millis() - startTime < 5000) {
            if (GPSSerial.available()) {
                uint8_t c = GPSSerial.read();
                
                // Log each byte
                Serial.printf("Byte: 0x%02X ", c);
                if (c >= 32 && c <= 126) {
                    Serial.printf("('%c')", (char)c);
                }
                Serial.println();
                
                // Check for NMEA start
                if (c == '$') {
                    dollarCount++;
                }
                
                // Check for UBlox sync chars
                if (c == UBLOX_SYNC1) {
                    ubxSync1Count++;
                }
                else if (c == UBLOX_SYNC2) {
                    if (buffer[bufferIndex-1] == UBLOX_SYNC1) {
                        ubxSync2Count++;
                    }
                }
                
                // Store in buffer
                if (bufferIndex < BUFFER_SIZE) {
                    buffer[bufferIndex++] = c;
                }
            }
        }
        
        Serial.printf("\nResults at %ld baud:\n", baudRates[i]);
        Serial.printf("NMEA starts ($) found: %d\n", dollarCount);
        Serial.printf("UBlox sync1 (0xB5) found: %d\n", ubxSync1Count);
        Serial.printf("UBlox sync2 pairs (0xB5,0x62) found: %d\n", ubxSync2Count);
        
        // Print part of the buffer for analysis
        Serial.println("\nFirst 32 bytes received:");
        for (int j = 0; j < min(32, bufferIndex); j++) {
            Serial.printf("0x%02X ", buffer[j]);
            if ((j + 1) % 8 == 0) Serial.println();
        }
        Serial.println();
        
        GPSSerial.end();
        bufferIndex = 0;
    }
}

void loop() {
    delay(1000);
}

#include <HardwareSerial.h>

// Try both pin configurations
struct PinConfig {
    int rx;
    int tx;
    const char* name;
} pinConfigs[] = {
    {16, 17, "Original: RX=16, TX=17"},
    {17, 16, "Swapped: RX=17, TX=16"}
};

// Common GPS baud rates, prioritizing 9600 which is most common
const long baudRates[] = {9600, 38400, 57600, 4800, 115200};
const int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);

HardwareSerial GPSSerial(1);
const int BUFFER_SIZE = 256;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

void testConfiguration(int rx, int tx, long baudRate) {
    Serial.printf("\n=== Testing RX=%d, TX=%d at %ld baud ===\n", rx, tx, baudRate);
    
    GPSSerial.begin(baudRate, SERIAL_8N1, rx, tx);
    delay(1000);
    
    // Clear buffer
    while(GPSSerial.available()) GPSSerial.read();
    
    unsigned long startTime = millis();
    int validStarts = 0;
    String currentSentence = "";
    
    while (millis() - startTime < 3000) {
        if (GPSSerial.available()) {
            char c = GPSSerial.read();
            
            if (c == '$') {
                if (!currentSentence.isEmpty()) {
                    Serial.printf("Incomplete: %s\n", currentSentence.c_str());
                }
                currentSentence = "$";
                validStarts++;
            } else if (!currentSentence.isEmpty()) {
                currentSentence += c;
                if (c == '\n' || c == '\r') {
                    Serial.printf("Complete: %s\n", currentSentence.c_str());
                    currentSentence = "";
                }
            }
            
            // Print character in multiple formats for debugging
            if (c >= 32 && c <= 126) {
                Serial.printf("Char: '%c' (ASCII: %d, Hex: 0x%02X)\n", c, c, (uint8_t)c);
            } else {
                Serial.printf("Char: <non-printable> (ASCII: %d, Hex: 0x%02X)\n", c, (uint8_t)c);
            }
        }
    }
    
    Serial.printf("Found %d sentence starts\n", validStarts);
    GPSSerial.end();
    delay(500);
}

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10);
    delay(2000);
    
    Serial.println("\nESP32 GPS Detector v3 - Testing all configurations");
    
    for (auto& pinConfig : pinConfigs) {
        Serial.printf("\n\n=== Testing %s ===\n", pinConfig.name);
        for (int i = 0; i < numBaudRates; i++) {
            testConfiguration(pinConfig.rx, pinConfig.tx, baudRates[i]);
        }
    }
    
    Serial.println("\nTest complete - check above for valid NMEA sentences");
}

void loop() {
    delay(1000);
}

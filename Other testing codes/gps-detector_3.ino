#include <HardwareSerial.h>

// Define GPS Serial pins
#define GPS_RX 16  // ESP32 pin GPIO16 connected to GPS TX
#define GPS_TX 17  // ESP32 pin GPIO17 connected to GPS RX

// Extended list of common baud rates to test
const long baudRates[] = {4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200};
const int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);

// GPS Serial interface
HardwareSerial GPSSerial(1);  // Use UART1

// Buffer for reading GPS data
const int BUFFER_SIZE = 256;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

// Function to check if a character is printable ASCII
bool isPrintableASCII(char c) {
    return (c >= 32 && c <= 126) || c == '\r' || c == '\n';
}

void detectBaudRate() {
    Serial.println("Starting enhanced baud rate detection...");
    
    for (int i = 0; i < numBaudRates; i++) {
        Serial.printf("\nTesting baud rate: %ld...\n", baudRates[i]);
        
        // Configure serial with different settings
        GPSSerial.end();
        delay(100);
        
        // Try different serial configurations
        const int numConfigs = 4;
        const uint32_t configs[numConfigs] = {
            SERIAL_8N1,
            SERIAL_8N2,
            SERIAL_7N1,
            SERIAL_7N2
        };
        
        for (int config = 0; config < numConfigs; config++) {
            Serial.printf("Testing with config %d...\n", config);
            GPSSerial.begin(baudRates[i], configs[config], GPS_RX, GPS_TX);
            
            // Clear any existing data
            while(GPSSerial.available()) {
                GPSSerial.read();
            }
            
            delay(100);
            
            // Variables for statistics
            int totalChars = 0;
            int printableChars = 0;
            int dollarSigns = 0;
            String sentence = "";
            
            // Try reading for a few seconds
            unsigned long startTime = millis();
            while (millis() - startTime < 3000) {
                if (GPSSerial.available()) {
                    char c = GPSSerial.read();
                    totalChars++;
                    
                    if (isPrintableASCII(c)) {
                        printableChars++;
                    }
                    
                    if (c == '$') {
                        dollarSigns++;
                        sentence = "$";  // Start collecting a new sentence
                    } else if (!sentence.isEmpty()) {
                        sentence += c;
                        if (c == '\n' || sentence.length() >= 80) {
                            Serial.printf("Potential NMEA sentence: %s", sentence.c_str());
                            sentence = "";
                        }
                    }
                    
                    // Print raw character for debugging
                    if (c >= 32 && c <= 126) {
                        Serial.printf("Received ASCII: '%c' (0x%02X)\n", c, (unsigned char)c);
                    } else {
                        Serial.printf("Received non-ASCII: 0x%02X\n", (unsigned char)c);
                    }
                }
            }
            
            // Analysis of received data
            if (totalChars > 0) {
                float printablePercent = (float)printableChars / totalChars * 100;
                Serial.printf("\nStatistics for %ld baud, config %d:\n", baudRates[i], config);
                Serial.printf("Total characters: %d\n", totalChars);
                Serial.printf("Printable characters: %d (%.1f%%)\n", printableChars, printablePercent);
                Serial.printf("Dollar signs found: %d\n", dollarSigns);
                
                // If we have a high percentage of printable characters and some $ signs,
                // this might be the correct rate
                if (printablePercent > 70 && dollarSigns > 0) {
                    Serial.printf("\nPOTENTIAL MATCH FOUND: %ld baud with config %d\n", 
                                baudRates[i], config);
                    Serial.println("Monitoring this configuration for valid NMEA sentences...");
                    
                    // Monitor for a few more seconds to confirm
                    startTime = millis();
                    while (millis() - startTime < 5000) {
                        if (GPSSerial.available()) {
                            Serial.write(GPSSerial.read());
                        }
                    }
                }
            }
            
            GPSSerial.end();
            delay(100);
        }
    }
}

void setup() {
    Serial.begin(115200);  // Start Serial Monitor
    while (!Serial) delay(10);  // Wait for Serial Monitor
    delay(2000);  // Give GPS module time to start up
    
    Serial.println("\nESP32 GPS Baud Rate Detector - Enhanced Version");
    Serial.println("Make sure GPS module is powered and connected properly:");
    Serial.println("GPS VCC -> ESP32 5V");
    Serial.println("GPS GND -> ESP32 GND");
    Serial.println("GPS TX  -> ESP32 GPIO16 (RX)");
    Serial.println("GPS RX  -> ESP32 GPIO17 (TX)");
    
    detectBaudRate();
}

void loop() {
    delay(1000);
}

#include <HardwareSerial.h>

// Define GPS Serial pins
#define GPS_RX 16  // ESP32 pin GPIO16 connected to GPS TX
#define GPS_TX 17  // ESP32 pin GPIO17 connected to GPS RX

// Common baud rates to test
const long baudRates[] = {9600, 19200, 38400, 57600, 115200};
const int numBaudRates = sizeof(baudRates) / sizeof(baudRates[0]);

// GPS Serial interface
HardwareSerial GPSSerial(1);  // Use UART1

// Buffer for reading GPS data
const int BUFFER_SIZE = 256;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

void detectBaudRate() {
  Serial.println("Starting baud rate detection with debug output...");
  
  for (int i = 0; i < numBaudRates; i++) {
    Serial.printf("\nTesting baud rate: %ld...\n", baudRates[i]);
    
    GPSSerial.begin(baudRates[i], SERIAL_8N1, GPS_RX, GPS_TX);
    
    // Try reading for a few seconds
    unsigned long startTime = millis();
    bool dataReceived = false;
    
    while (millis() - startTime < 5000) {  // Test for 5 seconds
      if (GPSSerial.available()) {
        char c = GPSSerial.read();
        dataReceived = true;
        
        // Print raw character in both ASCII and hex
        Serial.printf("Received: '%c' (0x%02X)\n", c, (unsigned char)c);
        
        // Collect characters into potential NMEA sentence
        if (c == '$') {  // Start of NMEA sentence
          bufferIndex = 0;
        }
        
        if (bufferIndex < BUFFER_SIZE - 1) {
          buffer[bufferIndex++] = c;
          
          if (c == '\n' || c == '\r') {  // End of sentence
            buffer[bufferIndex] = '\0';
            Serial.printf("Complete line received: %s", buffer);
          }
        }
      }
    }
    
    if (!dataReceived) {
      Serial.println("No data received at this baud rate");
    }
    
    GPSSerial.end();  // Clean up before trying next baud rate
    delay(1000);  // Wait a bit before trying next baud rate
  }
  
  Serial.println("\nBaud rate detection completed");
}

void setup() {
  Serial.begin(115200);  // Start Serial Monitor
  delay(1000);
  
  Serial.println("ESP32 GPS Baud Rate Detector - Debug Version");
  
  // Try swapping RX/TX pins in case they're reversed
  Serial.println("\nTrying original pin configuration...");
  detectBaudRate();
  
  // Swap RX and TX pins
  Serial.println("\nTrying swapped pins configuration...");
  #define TEMP_PIN GPS_RX
  #undef GPS_RX
  #define GPS_RX GPS_TX
  #undef GPS_TX
  #define GPS_TX TEMP_PIN
  detectBaudRate();
}

void loop() {
  // Nothing in loop - we just run the detection once
  delay(1000);
}

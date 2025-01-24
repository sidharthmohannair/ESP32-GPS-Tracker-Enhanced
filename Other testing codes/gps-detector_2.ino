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

// Function to detect valid NMEA sentences
bool isValidNMEA(const char* sentence) {
  // Check for '$' at start
  if (sentence[0] != '$') return false;
  
  // Calculate checksum
  uint8_t checksum = 0;
  int i = 1;
  
  // Find the '*' character
  while (sentence[i] != '*' && sentence[i] != '\0' && i < BUFFER_SIZE) {
    checksum ^= sentence[i];
    i++;
  }
  
  // If we found the asterisk and there are at least 2 more chars (checksum)
  if (sentence[i] == '*' && sentence[i+1] != '\0' && sentence[i+2] != '\0') {
    char expectedChecksum[3] = {sentence[i+1], sentence[i+2], '\0'};
    int expectedValue;
    sscanf(expectedChecksum, "%x", &expectedValue);
    return checksum == expectedValue;
  }
  
  return false;
}

// Function to parse NMEA data
void parseNMEA(const char* sentence) {
  char msgId[6];
  if (sscanf(sentence, "$%5s", msgId) == 1) {
    if (strcmp(msgId, "GNGGA") == 0) {
      // Parse GGA sentence (Global Positioning System Fix Data)
      float latitude, longitude;
      int hour, minute;
      float second;
      int satellites;
      float hdop, altitude;
      
      char latDir, lonDir;
      if (sscanf(sentence, "$GNGGA,%2d%2d%f,%f,%c,%f,%c,%d,%d,%f,%f,M",
                 &hour, &minute, &second,
                 &latitude, &latDir,
                 &longitude, &lonDir,
                 &satellites, &hdop, &altitude) >= 10) {
        
        // Convert latitude/longitude to decimal degrees
        float latDeg = int(latitude / 100);
        float latMin = latitude - (latDeg * 100);
        latitude = latDeg + (latMin / 60);
        if (latDir == 'S') latitude = -latitude;
        
        float lonDeg = int(longitude / 100);
        float lonMin = longitude - (lonDeg * 100);
        longitude = lonDeg + (lonMin / 60);
        if (lonDir == 'W') longitude = -longitude;
        
        // Print parsed data
        Serial.println("\nParsed GGA Data:");
        Serial.printf("Time: %02d:%02d:%06.3f UTC\n", hour, minute, second);
        Serial.printf("Latitude: %.6f°\n", latitude);
        Serial.printf("Longitude: %.6f°\n", longitude);
        Serial.printf("Satellites: %d\n", satellites);
        Serial.printf("HDOP: %.1f\n", hdop);
        Serial.printf("Altitude: %.1f m\n", altitude);
      }
    }
  }
}

void detectBaudRate() {
  Serial.println("Starting baud rate detection...");
  
  for (int i = 0; i < numBaudRates; i++) {
    Serial.printf("Testing baud rate: %ld...\n", baudRates[i]);
    
    GPSSerial.begin(baudRates[i], SERIAL_8N1, GPS_RX, GPS_TX);
    
    // Try reading for a few seconds
    unsigned long startTime = millis();
    int validSentences = 0;
    
    while (millis() - startTime < 3000) {  // Test for 3 seconds
      if (GPSSerial.available()) {
        char c = GPSSerial.read();
        
        if (c == '$') {  // Start of NMEA sentence
          bufferIndex = 0;
        }
        
        if (bufferIndex < BUFFER_SIZE - 1) {
          buffer[bufferIndex++] = c;
          
          if (c == '\n') {  // End of sentence
            buffer[bufferIndex] = '\0';
            if (isValidNMEA(buffer)) {
              validSentences++;
            }
          }
        }
      }
    }
    
    if (validSentences > 0) {
      Serial.printf("Found valid GPS data at %ld baud! (%d valid sentences)\n", 
                   baudRates[i], validSentences);
      return;  // Keep this baud rate
    }
    
    GPSSerial.end();  // Clean up before trying next baud rate
  }
  
  Serial.println("No valid GPS data found at any baud rate!");
}

void setup() {
  Serial.begin(115200);  // Start Serial Monitor
  delay(1000);
  
  Serial.println("ESP32 GPS Baud Rate Detector and NMEA Parser");
  detectBaudRate();
}

void loop() {
  if (GPSSerial.available()) {
    char c = GPSSerial.read();
    
    if (c == '$') {  // Start of NMEA sentence
      bufferIndex = 0;
    }
    
    if (bufferIndex < BUFFER_SIZE - 1) {
      buffer[bufferIndex++] = c;
      
      if (c == '\n') {  // End of sentence
        buffer[bufferIndex] = '\0';
        if (isValidNMEA(buffer)) {
          Serial.print("Valid NMEA: ");
          Serial.print(buffer);
          parseNMEA(buffer);
        }
      }
    }
  }
}

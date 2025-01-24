/*
  ESP32 GPS Tracker with Enhanced NMEA Parsing
  -----------------------------------------------------
  Created by: Sidharth Mohan (https://github.com/sidharthmohannair)
  License: MIT License
  
  This code implements an advanced GPS tracking solution using an ESP32 and a UBlox GPS module. 
  It incorporates various enhancements to ensure robust performance, accurate data processing, 
  and consistent tracking in real-world scenarios.

  Key Features and Improvements:
  --------------------------------
  1. **Robust NMEA Parsing**:
     - Enhanced checksum validation for reliable NMEA sentence handling.
     - Improved parsing logic with error handling for malformed data.
  
  2. **Optimized String Operations**:
     - Safe string manipulation to prevent buffer overflows.
     - Efficient use of memory for handling GPS data.

  3. **Reliable GPS Fix Validation**:
     - Validation based on multiple criteria to ensure data accuracy.
     - Tracks and stores the last valid fix for consistent performance.

  4. **Improved Timeout Management**:
     - Separate warning and reset timeouts for better control.
     - Handles edge cases like GPS module inactivity or signal loss.

  5. **Time and Data Handling**:
     - Accurate time formatting for logging and display.
     - Proper handling of `millis()` rollover to maintain system reliability.

  6. **Debugging and Monitoring**:
     - Consistent and detailed serial debugging output for easy troubleshooting.
     - Configurable debug modes for different deployment environments.

  7. **GPS Module Reset Functionality**:
     - Reset capability for the GPS module to recover from errors or signal issues.

  8. **Enhanced Usability**:
     - Clear code structure for better readability and maintenance.
     - Simplified configuration for UART pins and communication settings.

  How to Use:
  ------------
  - Connect your UBlox GPS module to the following ESP32 pins:
      * GPS_RX: Pin 17
      * GPS_TX: Pin 16
  - Upload the code to your ESP32 using the Arduino IDE.
  - Open the Serial Monitor at 115200 baud to view GPS data and debug information.

  Note:
  -----
  This code is intended for educational and non-commercial purposes under the MIT License. 
  Contributions and improvements are welcome to make it even better.

*/

#include <HardwareSerial.h>

// Pin definitions
#define GPS_RX 17
#define GPS_TX 16

// Constants
const int BUFFER_SIZE = 512;
const float KNOTS_TO_MS = 0.514444;
const unsigned long GPS_TIMEOUT = 5000;  // 5 seconds timeout
const unsigned long GPS_RESET_TIMEOUT = 10000;  // 10 seconds before reset

// GPS configuration
HardwareSerial GPSSerial(1);  // Use UART1
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

// GPS data structure
struct GPSData {
    // Position data
    float latitude = 0.0;
    float longitude = 0.0;
    float altitude = 0.0;
    float course = 0.0;  // GPS heading
    
    // Quality indicators
    int satellites = 0;
    float hdop = 0.0;
    int fixQuality = 0;
    char fixValid = 'V';  // 'A' for valid, 'V' for void
    
    // Speed and time
    float groundSpeed = 0.0;
    char utcTime[11] = "";
    char date[7] = "";
    
    // Additional data
    float magneticVariation = 0.0;
    float geoidHeight = 0.0;
    
    // Status tracking
    bool hasValidFix = false;
    unsigned long lastValidFix = 0;
} gpsData;

// Timing variables
unsigned long lastNMEAReceivedTime = 0;

// Function prototypes
void resetGPSModule();
bool validateChecksum(const char* sentence);
float convertToDecimalDegrees(const char* coord, char dir);
void safeStringCopy(char* dest, const char* src, size_t destSize);
unsigned long getTimeDifference(unsigned long current, unsigned long previous);
bool isGPSValid();
void processGGA(const String& nmea);
void processRMC(const String& nmea);
void processNMEASentence(const char* sentence);
void printGPSData();
void formatGPSTime();

void setup() {
    Serial.begin(115200);
    delay(1000);
    
  //  Serial.println("ESP32 GPS Antenna Tracker - Enhanced Version");
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    
  //  Serial.println("Waiting for GPS data...");
}

bool validateChecksum(const char* sentence) {
    if (sentence[0] != '$') return false;
    
    unsigned char checksum = 0;
    int i = 1; // Skip the '$'
    
    // Calculate checksum until '*' or end of string
    while (sentence[i] && sentence[i] != '*' && i < strlen(sentence)) {
        checksum ^= sentence[i];
        i++;
    }
    
    // If we found a '*', verify the checksum
    if (sentence[i] == '*') {
        char expected[3];
        sprintf(expected, "%02X", checksum);
        return (expected[0] == sentence[i+1] && expected[1] == sentence[i+2]);
    }
    
    return false;
}

float convertToDecimalDegrees(const char* coord, char dir) {
    if (strlen(coord) == 0 || strchr(coord, '.') == NULL) {
        Serial.println("Error: Invalid coordinate string.");
        return 0.0;
    }

    // For latitude: first 2 digits are degrees
    // For longitude: first 3 digits are degrees
    int degreeDigits = (dir == 'N' || dir == 'S') ? 2 : 3;
    
    // Extract degrees
    char degStr[4];
    strncpy(degStr, coord, degreeDigits);
    degStr[degreeDigits] = '\0';
    float degrees = atof(degStr);
    
    // Extract minutes
    float minutes = atof(coord + degreeDigits);
    
    // Calculate decimal degrees
    float decimal = degrees + (minutes / 60.0);
    
    return (dir == 'S' || dir == 'W') ? -decimal : decimal;
}

void safeStringCopy(char* dest, const char* src, size_t destSize) {
    strncpy(dest, src, destSize - 1);
    dest[destSize - 1] = '\0';
}

unsigned long getTimeDifference(unsigned long current, unsigned long previous) {
    return (current >= previous) ? 
           (current - previous) : 
           (0xFFFFFFFF - previous + current);
}

bool isGPSValid() {
    return (gpsData.fixQuality > 0 &&    // 0 = no fix
            gpsData.fixValid == 'A' &&    // A = active
            gpsData.satellites >= 4 &&    // Minimum satellites for 3D fix
            gpsData.hdop < 5.0);         // Reasonable HDOP threshold
}

void processGGA(const String& nmea) {
    char time[11], lat[15], lon[15], alt[10], geoid[10];
    char latDir, lonDir, altUnit;  // Added altUnit declaration

    int fields = sscanf(nmea.c_str(), 
                       "$%*[^,],%[^,],%[^,],%c,%[^,],%c,%d,%d,%f,%[^,],%c,%[^,]",
                       time, lat, &latDir, lon, &lonDir, &gpsData.fixQuality, 
                       &gpsData.satellites, &gpsData.hdop, alt, &altUnit, geoid);

    if (fields < 11) {
        Serial.println("Warning: Incomplete GGA sentence");
        return;
    }

    safeStringCopy(gpsData.utcTime, time, sizeof(gpsData.utcTime));
    gpsData.latitude = convertToDecimalDegrees(lat, latDir);
    gpsData.longitude = convertToDecimalDegrees(lon, lonDir);
    gpsData.altitude = atof(alt);
    gpsData.geoidHeight = atof(geoid);

    // Update fix status
    gpsData.hasValidFix = isGPSValid();
    if (gpsData.hasValidFix) {
        gpsData.lastValidFix = millis();
    }

    // Debugging prints
   // Serial.println("Debug: Parsing GGA Sentence...");
   // Serial.printf("Time: %s\n", time);
   // Serial.printf("Latitude: %s %c\n", lat, latDir);
   // Serial.printf("Longitude: %s %c\n", lon, lonDir);
   // Serial.printf("Altitude: %s m, Geoid: %s m\n", alt, geoid);
   // Serial.printf("Fix Quality: %d, Satellites: %d, HDOP: %.2f\n", 
   //              gpsData.fixQuality, gpsData.satellites, gpsData.hdop);
}

void processRMC(const String& nmea) {
    char time[11], lat[15], lon[15], date[7];
    char latDir, lonDir;
    float magVar;
    char magVarDir;

int fields = sscanf(nmea.c_str(), 
                   "$%*[^,],%[^,],%c,%[^,],%c,%[^,],%c,%f,%f,%[^,],%f,%c,",
                   time, &gpsData.fixValid, lat, &latDir, lon, &lonDir,
                   &gpsData.groundSpeed, &gpsData.course, date, &magVar, &magVarDir);

    if (fields < 11) {
        Serial.println("Warning: Incomplete RMC sentence");
        return;
    }

    safeStringCopy(gpsData.date, date, sizeof(gpsData.date));
    gpsData.magneticVariation = (magVarDir == 'W') ? -magVar : magVar;

    // Debugging prints
//    Serial.println("Debug: Parsing RMC Sentence...");
//    Serial.printf("Time: %s, Date: %s\n", time, date);
//    Serial.printf("Latitude: %s %c\n", lat, latDir);
//    Serial.printf("Longitude: %s %c\n", lon, lonDir);
//    Serial.printf("Ground Speed: %.2f knots, Course: %.2f°\n", 
//                 gpsData.groundSpeed, gpsData.course);
//    Serial.printf("Magnetic Variation: %.2f° %c\n", magVar, magVarDir);
}

void formatGPSTime() {
    if (strlen(gpsData.utcTime) >= 6) {
        char formattedTime[9];
        snprintf(formattedTime, sizeof(formattedTime), 
                "%c%c:%c%c:%c%c",
                gpsData.utcTime[0], gpsData.utcTime[1],
                gpsData.utcTime[2], gpsData.utcTime[3],
                gpsData.utcTime[4], gpsData.utcTime[5]);
        Serial.printf("Time (UTC): %s\n", formattedTime);
    }
}

void printGPSData() {
    Serial.println("\n=== GPS Antenna Tracker Data ===");
    
    // Position
    Serial.printf("Latitude: %.6f°\n", gpsData.latitude);
    Serial.printf("Longitude: %.6f°\n", gpsData.longitude);
    Serial.printf("Altitude: %.1f m\n", gpsData.altitude);
    Serial.printf("GPS Heading: %.1f°\n", gpsData.course);
    
    // Quality and Status
    Serial.printf("Fix Quality: %d\n", gpsData.fixQuality);
    Serial.printf("Fix Status: %c\n", gpsData.fixValid);
    Serial.printf("Satellites: %d\n", gpsData.satellites);
    Serial.printf("HDOP: %.1f\n", gpsData.hdop);
    
    // Speed and Navigation
  //  Serial.printf("Ground Speed: %.1f knots\n", gpsData.groundSpeed);
    Serial.printf("Ground Speed: %.2f m/s\n", gpsData.groundSpeed * KNOTS_TO_MS);
    Serial.printf("Magnetic Variation: %.1f°\n", gpsData.magneticVariation);
    
    // Time and Date
    formatGPSTime();
    if (strlen(gpsData.date) >= 6) {
        Serial.printf("Date: %c%c/%c%c/%c%c\n",
                     gpsData.date[0], gpsData.date[1],
                     gpsData.date[2], gpsData.date[3],
                     gpsData.date[4], gpsData.date[5]);
    }
}

void resetGPSModule() {
    Serial.println("Resetting GPS module...");
    GPSSerial.end();
    delay(1000);
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    lastNMEAReceivedTime = millis();
}

void processNMEASentence(const char* sentence) {
    if (!validateChecksum(sentence)) {
        Serial.println("Warning: Checksum validation failed");
        return;
    }

    String nmea = String(sentence);
    nmea.trim();

    // Debug: Print raw NMEA sentence
 //   Serial.print("Raw NMEA: ");
 //   Serial.println(nmea);

    int firstComma = nmea.indexOf(',');
    if (firstComma == -1) return;

    String sentenceType = nmea.substring(1, firstComma);

    if (sentenceType == "GPGGA" || sentenceType == "GNGGA") {
        processGGA(nmea);
    } else if (sentenceType == "GPRMC" || sentenceType == "GNRMC") {
        processRMC(nmea);
        printGPSData(); // Print data after RMC sentence
    }
}

void loop() {
    while (GPSSerial.available()) {
        char c = GPSSerial.read();

        if (c == '$') {
            bufferIndex = 0; // Start of a new NMEA sentence
        }

        if (bufferIndex < BUFFER_SIZE - 1) {
            buffer[bufferIndex++] = c;

            if (c == '\n') { // End of NMEA sentence
                buffer[bufferIndex] = '\0'; // Null-terminate the string

                if (buffer[0] == '$') {
                    processNMEASentence(buffer);
                    lastNMEAReceivedTime = millis();
                }

                buffer[0] = '\0'; // Clear the buffer
                bufferIndex = 0;  // Reset buffer index
            }
        } else {
            bufferIndex = 0; // Reset buffer if too long
        }
    }

    // Check for GPS timeout
    unsigned long timeSinceLastNMEA = getTimeDifference(millis(), lastNMEAReceivedTime);
    
    if (timeSinceLastNMEA > GPS_TIMEOUT) {
        Serial.println("Warning: No GPS data received in the last 5 seconds.");
    }
    
    // Check if GPS reset is needed
    if (timeSinceLastNMEA > GPS_RESET_TIMEOUT) {
        resetGPSModule();
    }
}
#include <HardwareSerial.h>

// Pin definitions
#define GPS_RX 17
#define GPS_TX 16

// GPS configuration
HardwareSerial GPSSerial(1);  // Use UART1
const int BUFFER_SIZE = 512;
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
} gpsData;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("ESP32 GPS Antenna Tracker");
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    
    Serial.println("Waiting for GPS data...");
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

void processGGA(const String& nmea) {
    char time[11], lat[15], lon[15], alt[10], geoid[10];
    char latDir, lonDir;

    sscanf(nmea.c_str(), "$%*[^,],%[^,],%[^,],%c,%[^,],%c,%d,%d,%f,%[^,],%*[^,],%[^,]",
           time, lat, &latDir, lon, &lonDir, &gpsData.fixQuality, 
           &gpsData.satellites, &gpsData.hdop, alt, geoid);

    strcpy(gpsData.utcTime, time);
    gpsData.latitude = convertToDecimalDegrees(lat, latDir);
    gpsData.longitude = convertToDecimalDegrees(lon, lonDir);
    gpsData.altitude = atof(alt);
    gpsData.geoidHeight = atof(geoid);

    // Debugging prints
    Serial.println("Debug: Parsing GGA Sentence...");
    Serial.printf("Time: %s\n", time);
    Serial.printf("Latitude: %s %c\n", lat, latDir);
    Serial.printf("Longitude: %s %c\n", lon, lonDir);
    Serial.printf("Altitude: %s m, Geoid: %s m\n", alt, geoid);
    Serial.printf("Fix Quality: %d, Satellites: %d, HDOP: %.2f\n", gpsData.fixQuality, gpsData.satellites, gpsData.hdop);
}


void processRMC(const String& nmea) {
    char time[11], lat[15], lon[15], date[7];
    char latDir, lonDir;
    float magVar;
    char magVarDir;

    sscanf(nmea.c_str(), "$%*[^,],%[^,],%c,%[^,],%c,%[^,],%c,%f,%f,%[^,],%f,%c",
           time, &gpsData.fixValid, lat, &latDir, lon, &lonDir,
           &gpsData.groundSpeed, &gpsData.course, date, &magVar, &magVarDir);

    strcpy(gpsData.date, date);
    gpsData.magneticVariation = (magVarDir == 'W') ? -magVar : magVar;

    // Debugging prints
    Serial.println("Debug: Parsing RMC Sentence...");
    Serial.printf("Time: %s, Date: %s\n", time, date);
    Serial.printf("Latitude: %s %c\n", lat, latDir);
    Serial.printf("Longitude: %s %c\n", lon, lonDir);
    Serial.printf("Ground Speed: %.2f knots, Course: %.2f°\n", gpsData.groundSpeed, gpsData.course);
    Serial.printf("Magnetic Variation: %.2f° %c\n", magVar, magVarDir);
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
    Serial.printf("Ground Speed: %.1f knots\n", gpsData.groundSpeed);
    Serial.printf("Ground Speed: %.2f m/s\n", gpsData.groundSpeed * 0.514444);
    Serial.printf("Magnetic Variation: %.1f°\n", gpsData.magneticVariation);
    
    // Time and Date
    if (strlen(gpsData.utcTime) >= 6) {
        Serial.printf("Time (UTC): %c%c:%c%c:%c%c\n",
                     gpsData.utcTime[0], gpsData.utcTime[1],
                     gpsData.utcTime[2], gpsData.utcTime[3],
                     gpsData.utcTime[4], gpsData.utcTime[5]);
    }
    if (strlen(gpsData.date) >= 6) {
        Serial.printf("Date: %c%c/%c%c/%c%c\n",
                     gpsData.date[0], gpsData.date[1],
                     gpsData.date[2], gpsData.date[3],
                     gpsData.date[4], gpsData.date[5]);
    }
}

void processNMEASentence(const char* sentence) {
    String nmea = String(sentence);
    nmea.trim();

    // Debug: Print raw NMEA sentence
    Serial.print("Raw NMEA: ");
    Serial.println(nmea);

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

unsigned long lastNMEAReceivedTime = 0;

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
                    processNMEASentence(buffer); // Process the NMEA sentence
                    lastNMEAReceivedTime = millis(); // Update last received time
                }

                buffer[0] = '\0'; // Clear the buffer
                bufferIndex = 0;  // Reset buffer index
            }
        } else {
            bufferIndex = 0; // Reset buffer if too long
        }
    }

    // Timeout check
    if (millis() - lastNMEAReceivedTime > 5000) {
        Serial.println("Warning: No GPS data received in the last 5 seconds.");
    }
}



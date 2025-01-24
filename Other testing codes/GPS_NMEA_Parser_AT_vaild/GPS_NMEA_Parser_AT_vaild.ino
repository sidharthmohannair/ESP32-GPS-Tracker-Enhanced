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

// Add this structure after the GPSData struct
struct LocationBounds {
    float minLat = 26.45;  // Minimum expected latitude
    float maxLat = 26.55;  // Maximum expected latitude
    float minLon = 80.18;  // Minimum expected longitude
    float maxLon = 80.28;  // Maximum expected longitude
    float maxAllowedHDOP = 2.0;  // Maximum acceptable HDOP
    int minSatellites = 4;  // Minimum satellites for reliable fix
} bounds;

bool isLocationValid() {
    // Check if coordinates are within expected bounds
    if (gpsData.latitude < bounds.minLat || gpsData.latitude > bounds.maxLat ||
        gpsData.longitude < bounds.minLon || gpsData.longitude > bounds.maxLon) {
        return false;
    }
    
    // Check quality indicators
    if (gpsData.hdop > bounds.maxAllowedHDOP) {
        return false;
    }
    
    if (gpsData.satellites < bounds.minSatellites) {
        return false;
    }
    
    // Check fix quality and status
    if (gpsData.fixQuality == 0 || gpsData.fixValid != 'A') {
        return false;
    }
    
    return true;
}


void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("ESP32 GPS Antenna Tracker");
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    
    Serial.println("Waiting for GPS data...");
}

float convertToDecimalDegrees(const char* coord, char dir) {
    float degrees = atof(coord) / 100.0;
    float minutes = fmod(atof(coord), 100.0);
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
}

void printGPSData() {
        Serial.println("\n=== GPS Antenna Tracker Data ===");
    
    // Check location validity
    bool valid = isLocationValid();
    Serial.printf("Location Valid: %s\n", valid ? "YES" : "NO");
    
    if (!valid) {
        Serial.println("WARNING: GPS data outside expected bounds!");
        Serial.println("Expected region: 26.45-26.55°N, 80.18-80.28°E");
    }
    
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
    
    int firstComma = nmea.indexOf(',');
    if (firstComma == -1) return;
    
    String sentenceType = nmea.substring(1, firstComma);
    
    if (sentenceType == "GPGGA" || sentenceType == "GNGGA") {
        processGGA(nmea);
    }
    else if (sentenceType == "GPRMC" || sentenceType == "GNRMC") {
        processRMC(nmea);
        // Print all data after RMC since it's typically sent last in the NMEA sequence
        printGPSData();
    }
}

void loop() {
    while (GPSSerial.available()) {
        char c = GPSSerial.read();
        
        if (c == '$') {
            bufferIndex = 0;
        }
        
        if (bufferIndex < BUFFER_SIZE - 1) {
            buffer[bufferIndex++] = c;
            
            if (c == '\n') {
                buffer[bufferIndex] = '\0';
                if (buffer[0] == '$') {
                    processNMEASentence(buffer);
                }
                bufferIndex = 0;
            }
        } else {
            bufferIndex = 0;
        }
    }
}
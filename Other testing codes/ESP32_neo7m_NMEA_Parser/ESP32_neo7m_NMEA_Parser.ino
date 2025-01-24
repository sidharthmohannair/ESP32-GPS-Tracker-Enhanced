/*
This code will:

Read GPS data at 9600 baud
Parse both GGA (position fix) and RMC (navigation) sentences
Convert the raw data into a readable format
Show:

Time and date
Latitude and longitude in decimal degrees
Number of satellites
Fix quality
Altitude
Speed and course when available

This code is created by Sidharth Mohan





*/




#include <HardwareSerial.h>

#define GPS_RX 17
#define GPS_TX 16

HardwareSerial GPSSerial(1);  // Use UART1
const int BUFFER_SIZE = 512;
char buffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
    Serial.begin(115200);  // Serial Monitor
    delay(1000);
    
    Serial.println("ESP32 GPS NMEA Parser");
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    
    Serial.println("Waiting for GPS data...");
}

void processNMEASentence(const char* sentence) {
    // Remove \r\n if present
    String nmea = String(sentence);
    nmea.trim();
    
    // Extract sentence type (first field between $ and first comma)
    int firstComma = nmea.indexOf(',');
    if (firstComma == -1) return;
    
    String sentenceType = nmea.substring(1, firstComma);
    Serial.printf("\nSentence Type: %s\n", sentenceType.c_str());
    
    // Process different NMEA sentence types
    if (sentenceType == "GPGGA" || sentenceType == "GNGGA") {
        // GGA - Global Positioning System Fix Data
        char time[11], lat[15], lon[15], alt[10];
        char latDir, lonDir;
        int quality, satellites;
        float hdop;
        
        sscanf(nmea.c_str(), "$%*[^,],%[^,],%[^,],%c,%[^,],%c,%d,%d,%f,%[^,]",
               time, lat, &latDir, lon, &lonDir, &quality, &satellites, &hdop, alt);
        
        Serial.println("=== GPS Fix Data ===");
        
        // Format time from HHMMSS.SS to HH:MM:SS
        if (strlen(time) >= 6) {
            Serial.printf("Time (UTC): %c%c:%c%c:%c%c\n",
                         time[0], time[1], time[2], time[3], time[4], time[5]);
        }
        
        // Convert lat/lon to decimal degrees
        if (strlen(lat) > 0 && strlen(lon) > 0) {
            float latDeg = atof(lat) / 100;
            float latMin = fmod(atof(lat), 100);
            float latitude = latDeg + (latMin / 60);
            if (latDir == 'S') latitude = -latitude;
            
            float lonDeg = atof(lon) / 100;
            float lonMin = fmod(atof(lon), 100);
            float longitude = lonDeg + (lonMin / 60);
            if (lonDir == 'W') longitude = -longitude;
            
            Serial.printf("Latitude: %.6f°\n", latitude);
            Serial.printf("Longitude: %.6f°\n", longitude);
        }
        
        Serial.printf("Fix Quality: %d\n", quality);
        Serial.printf("Satellites: %d\n", satellites);
        Serial.printf("HDOP: %.1f\n", hdop);
        Serial.printf("Altitude: %s meters\n", alt);
    }
    else if (sentenceType == "GPRMC" || sentenceType == "GNRMC") {
        // RMC - Recommended Minimum Navigation Information
        char time[11], status, lat[15], latDir, lon[15], lonDir, date[7];
        float speed, course;
        
        sscanf(nmea.c_str(), "$%*[^,],%[^,],%c,%[^,],%c,%[^,],%c,%f,%f,%[^,]",
               time, &status, lat, &latDir, lon, &lonDir, &speed, &course, date);
        
        Serial.println("=== Navigation Data ===");
        if (status == 'A') {
            Serial.println("Status: Active (Valid)");
        } else if (status == 'V') {
            Serial.println("Status: Void (Invalid)");
        }
        
        // Format date from DDMMYY to DD/MM/YY
        if (strlen(date) >= 6) {
            Serial.printf("Date: %c%c/%c%c/%c%c\n",
                         date[0], date[1], date[2], date[3], date[4], date[5]);
        }
        
        Serial.printf("Speed (knots): %.1f\n", speed);
        Serial.printf("Course: %.1f°\n", course);
    }
}

void loop() {
    while (GPSSerial.available()) {
        char c = GPSSerial.read();
        
        if (c == '$') {  // Start of new NMEA sentence
            bufferIndex = 0;
        }
        
        if (bufferIndex < BUFFER_SIZE - 1) {
            buffer[bufferIndex++] = c;
            
            if (c == '\n') {  // End of sentence
                buffer[bufferIndex] = '\0';
                if (buffer[0] == '$') {
                    processNMEASentence(buffer);
                }
                bufferIndex = 0;
            }
        } else {
            bufferIndex = 0;  // Buffer overflow protection
        }
    }
}
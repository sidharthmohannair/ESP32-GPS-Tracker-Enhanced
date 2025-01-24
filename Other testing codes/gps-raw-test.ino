#include <HardwareSerial.h>

// Define pins
#define GPS_RX 16
#define GPS_TX 17

// GPS Serial interface
HardwareSerial GPSSerial(1);  // Use UART1

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("ESP32 GPS Raw Data Test");
    Serial.println("Trying 9600 baud...");
    
    // Start with most common baud rate
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
    
    // Print pin configuration
    Serial.println("\nConnections to check:");
    Serial.println("GPS VCC -> ESP32 5V");
    Serial.println("GPS GND -> ESP32 GND");
    Serial.println("GPS TX  -> ESP32 GPIO16 (RX)");
    Serial.println("GPS RX  -> ESP32 GPIO17 (TX)");
    
    Serial.println("\nStarting raw data monitor...");
    Serial.println("You should see some data within 5-10 seconds if connections are correct");
}

void loop() {
    // Print raw bytes in both ASCII and HEX
    while (GPSSerial.available()) {
        byte b = GPSSerial.read();
        // Print in multiple formats
        Serial.print("Received byte: ");
        if (b >= 32 && b <= 126) {  // Printable ASCII
            Serial.print("ASCII '");
            Serial.print((char)b);
            Serial.print("' | ");
        } else {
            Serial.print("Non-printable | ");
        }
        Serial.print("DEC ");
        Serial.print(b);
        Serial.print(" | HEX 0x");
        Serial.println(b < 16 ? "0" : "");
        Serial.println(b, HEX);
    }
    
    // Print a status message every 5 seconds if no data
    static unsigned long lastMsg = 0;
    if (millis() - lastMsg > 5000) {
        Serial.println("Waiting for GPS data... (check connections if no data appears)");
        lastMsg = millis();
    }
}

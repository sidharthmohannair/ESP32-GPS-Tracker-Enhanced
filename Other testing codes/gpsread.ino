#include <TinyGPS++.h>
#include <WiFi.h>
#include <WebServer.h>

// Replace with your network credentials
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";

// GPS Module connection
#define GPS_RX 16  // GPS TX connects to ESP32 RX
#define GPS_TX 17  // GPS RX connects to ESP32 TX

WebServer server(80);
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);  // Use UART1

String latitude = "0";
String longitude = "0";
String altitude = "0";
String satellites = "0";
String speed_kmh = "0";
String hdop = "0";

void setup() {
  Serial.begin(115200);
  
  // Initialize GPS serial
  GPSSerial.begin(460800, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
}

void loop() {
  // Read GPS data
  while (GPSSerial.available() > 0) {
    if (gps.encode(GPSSerial.read())) {
      updateGPSData();
    }
  }
  
  server.handleClient();
}

void updateGPSData() {
  if (gps.location.isValid()) {
    latitude = String(gps.location.lat(), 6);
    longitude = String(gps.location.lng(), 6);
  }
  
  if (gps.altitude.isValid()) {
    altitude = String(gps.altitude.meters(), 2);
  }
  
  if (gps.satellites.isValid()) {
    satellites = String(gps.satellites.value());
  }
  
  if (gps.speed.isValid()) {
    speed_kmh = String(gps.speed.kmph(), 2);
  }
  
  if (gps.hdop.isValid()) {
    hdop = String(gps.hdop.hdop(), 2);
  }
}

void handleRoot() {
  String html = R"(
    <!DOCTYPE html>
    <html>
    <head>
      <title>Here+ GPS Data</title>
      <meta name='viewport' content='width=device-width, initial-scale=1.0'>
      <style>
        body { font-family: Arial; margin: 20px; }
        .data-container { margin: 20px; padding: 15px; border: 1px solid #ccc; }
        .refresh-button { padding: 10px; margin: 10px; }
      </style>
      <script>
        function updateData() {
          fetch('/data')
            .then(response => response.json())
            .then(data => {
              document.getElementById('lat').textContent = data.latitude;
              document.getElementById('lng').textContent = data.longitude;
              document.getElementById('alt').textContent = data.altitude;
              document.getElementById('sat').textContent = data.satellites;
              document.getElementById('spd').textContent = data.speed;
              document.getElementById('hdp').textContent = data.hdop;
            });
        }
        
        setInterval(updateData, 1000);
      </script>
    </head>
    <body>
      <h2>Here+ GPS Data Visualization</h2>
      <div class='data-container'>
        <p>Latitude: <span id='lat'>--</span>°</p>
        <p>Longitude: <span id='lng'>--</span>°</p>
        <p>Altitude: <span id='alt'>--</span> m</p>
        <p>Satellites: <span id='sat'>--</span></p>
        <p>Speed: <span id='spd'>--</span> km/h</p>
        <p>HDOP: <span id='hdp'>--</span></p>
      </div>
      <button class='refresh-button' onclick='updateData()'>Refresh Data</button>
    </body>
    </html>
  )";
  server.send(200, "text/html", html);
}

void handleData() {
  String jsonData = "{";
  jsonData += "\"latitude\":\"" + latitude + "\",";
  jsonData += "\"longitude\":\"" + longitude + "\",";
  jsonData += "\"altitude\":\"" + altitude + "\",";
  jsonData += "\"satellites\":\"" + satellites + "\",";
  jsonData += "\"speed\":\"" + speed_kmh + "\",";
  jsonData += "\"hdop\":\"" + hdop + "\"";
  jsonData += "}";
  
  server.send(200, "application/json", jsonData);
}

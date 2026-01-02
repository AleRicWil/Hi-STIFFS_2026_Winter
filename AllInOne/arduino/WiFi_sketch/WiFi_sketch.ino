#include <WiFi.h>       // For WiFi functionality
#include <ArduinoOTA.h> // For Over-The-Air updates
WiFiServer server(80);  // Simple web server on port 80

// Customize these for your setup
const char* ssid = "Hi-STIFFS_Nano";       // WiFi network name (SSID) - choose something unique
const char* password = "BYUCropBio";    // WiFi password (minimum 8 characters)
const char* ota_password = "BYUCropBio";    // Optional OTA password for security (change this)

void setup() {
  Serial.begin(115200);  // Start serial for debugging (view in Serial Monitor)
  int startTime = millis();
  while (!Serial) {
    if (millis() - startTime > 2000) {
      Serial.println("No Serial Connection. Proceeding...");
      break;
    }
  }
  Serial.println("Starting SoftAP (wireless access point)...");

  // Set up SoftAP mode (Nano creates its own WiFi network)
  WiFi.softAP(ssid, password);
  
  // Set up web connection
  server.begin();  // Start the server
  Serial.println("Web server started for mock data");
  
  // Print IP address (default is 192.168.4.1)
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());

  // Set up OTA
  ArduinoOTA.setHostname(ssid);  // Device name for OTA discovery
  ArduinoOTA.setPassword(ota_password);    // Secure OTA with password
  ArduinoOTA.begin();                      // Start OTA service

  Serial.println("OTA ready. Connect to WiFi and use OTA for updates.");
}

void loop() {
  ArduinoOTA.handle();  // Handle OTA requests in the background
  Serial.println("OTA Loop.");

  WiFiClient client = server.available();  // Check for client connection
  if (client) {  // If a new client connects
    Serial.println("New Client.");
    String currentLine = "";  // Hold incoming data
    while (client.connected()) {  // Loop while connected
      if (client.available()) {   // If bytes to read
        char c = client.read();   // Read byte
        Serial.write(c);          // Debug to serial
        if (c == '\n') {  // If newline
          // If current line is blank (end of request headers)
          if (currentLine.length() == 0) {
            // Send SSE headers to start streaming
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/event-stream");
            client.println("Cache-Control: no-cache");
            client.println("Connection: keep-alive");
            client.println();

            // Now enter streaming loop: send mock data every 1 second
            while (client.connected()) {
              // Generate and send mock data as SSE event
              unsigned long mock_ts = micros() / 1000000UL;  // Mock timestamp
              int32_t mock_A1 = random(-1000000, 1000000);   // Mock raw values
              int32_t mock_A2 = random(-1000000, 1000000);
              client.print(mock_ts);
              client.print(",");
              client.print(mock_A1);
              client.print(",");
              client.println(mock_A2);
              client.println();  // Double newline ends SSE event
              client.flush();    // Ensure data is sent immediately

              delay(1);  // Wait 1 millisecond before next send
            }
            break;  // Exit if connection drops
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();  // Close connection when done
    Serial.println("Client disconnected.");
  }

  delay(1000);          // Small delay to avoid overloading the loop
}
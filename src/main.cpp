#include <HardwareSerial.h>
#include <DHT20.h>
#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

constexpr char WIFI_SSID[] = "nhatvu";
constexpr char WIFI_PASSWORD[] = "25122003";
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

constexpr char TOKEN[] = "1tqsqvh62gmcrj1yf0w7";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

#define SERIAL_DEBUG_BAUD 115200UL
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

DHT20 dht20;

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }
  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

void setup()
{
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();
  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();
}

void loop()
{
  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
  }
    
  dht20.read();
  float temperature = dht20.getTemperature();
  float humidity = dht20.getHumidity();

  if (isnan(temperature) || isnan(humidity))
  {
    Serial.println("Failed to read from DHT20 sensor!");
  }

  else
  {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity);
    Serial.println("%");
  }

  tb.sendTelemetryData("temperature", temperature);
  tb.sendTelemetryData("humidity", humidity);
  tb.sendTelemetryData("long", 106.80633605864662);
  tb.sendTelemetryData("lat", 10.880018410410052);

  tb.loop();
  delay(1000);
}

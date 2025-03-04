#include <HardwareSerial.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <DHT20.h>
#include <Wifi.h>

constexpr char WIFI_SSID[] = "camellia";
constexpr char WIFI_PASSWORD[] = "12345678912";
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

// Task handles
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;
TaskHandle_t Task3Handle = NULL;
TaskHandle_t Task4Handle = NULL;
TaskHandle_t Task5Handle = NULL;

// DHT20 Sensor
DHT20 DHT;

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

void Task1(void *pvParameters)
{
  while (1)
  {
    int status = DHT.read();  // > 1000
    float temperature = DHT.getTemperature();
    Serial.print("DHT20 Temperature: ");
    Serial.print(temperature, 1);
    Serial.println(" °C");
    tb.sendTelemetryData("temperature", temperature);

    Serial.print("Status: ");
    switch (status)
    {
    case DHT20_OK:
      Serial.println("OK");
      break;
    case DHT20_ERROR_CHECKSUM:
      Serial.println("Checksum error");
      break;
    case DHT20_ERROR_CONNECT:
      Serial.println("Connect error");
      break;
    case DHT20_MISSING_BYTES:
      Serial.println("Missing bytes");
      break;
    case DHT20_ERROR_BYTES_ALL_ZERO:
      Serial.println("All bytes read zero");
      break;
    case DHT20_ERROR_READ_TIMEOUT:
      Serial.println("Read time out");
      break;
    case DHT20_ERROR_LASTREAD:
      Serial.println("Read too fast");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
    Serial.println();
    vTaskDelay(pdMS_TO_TICKS(1500)); // Delay 1500ms
  }
}

void Task2(void *pvParameters)
{
  while (1)
  {
    float humidity = DHT.getHumidity();
    Serial.print("DHT20 Humidity: ");
    Serial.print(humidity, 1);
    Serial.println(" %");
    tb.sendTelemetryData("humidity", humidity);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1000ms
  }
}

void Task3(void *pvParameters)
{
  while (1)
  {
    Serial.println("MSSV: 2012458");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Delay 2000ms
  }
}

void Task4(void *pvParameters)
{
  while (1)
  {
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
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void Task5(void *pvParameters)
{
  while (1)
  {
    if (!reconnect()) {
      return;
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C

  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();
  DHT.begin();

  // Initialize DHT20
  if (!DHT.begin())
  {
    Serial.println("Failed to initialize DHT20 sensor!");
    while (1)
      ;
  }
  Serial.println("DHT20 sensor initialized.");

  // Create Tasks
  xTaskCreate(Task1, "Temperature", 2000, NULL, 1, &Task1Handle);
  xTaskCreate(Task2, "Humidity", 2000, NULL, 1, &Task2Handle);
  xTaskCreate(Task3, "MSSV", 1000, NULL, 1, &Task3Handle);
  xTaskCreate(Task4, "Check Connection Thingsboard", 2000, NULL, 1, &Task4Handle);
  xTaskCreate(Task5, "Check Wifi", 2000, NULL, 1, &Task4Handle);
}

void loop()
{
  // Empty - FreeRTOS handles tasks
}



// void Task1(void *pvParameters) {
//     while (1) {
//         Serial.println("Hello from Task1");
//         vTaskDelay(pdMS_TO_TICKS(1000));  // Delay 1000ms
//     }
// }
// 

// void Task2(void *pvParameters) {
//     while (1) {
//         Serial.println("Hello from Task2");
//         vTaskDelay(pdMS_TO_TICKS(1500));  // Delay 1500ms
//     }
// }

// Task 3: Read DHT20 Temperature & Humidity
// void Task3(void *pvParameters) {
//     while (1) {
//         if (millis() - DHT.lastRead() >= 2000) {
//             int status = DHT.read();

//             Serial.print("DHT20 Temperature: ");
//             Serial.print(DHT.getTemperature(), 1);
//             Serial.println(" °C");

//             Serial.print("DHT20 Humidity: ");
//             Serial.print(DHT.getHumidity(), 1);
//             Serial.println(" %");

//             Serial.print("Status: ");
//             switch (status) {
//                 case DHT20_OK:
//                     Serial.println("OK");
//                     break;
//                 case DHT20_ERROR_CHECKSUM:
//                     Serial.println("Checksum error");
//                     break;
//                 case DHT20_ERROR_CONNECT:
//                     Serial.println("Connect error");
//                     break;
//                 case DHT20_MISSING_BYTES:
//                     Serial.println("Missing bytes");
//                     break;
//                 case DHT20_ERROR_BYTES_ALL_ZERO:
//                     Serial.println("All bytes read zero");
//                     break;
//                 case DHT20_ERROR_READ_TIMEOUT:
//                     Serial.println("Read time out");
//                     break;
//                 case DHT20_ERROR_LASTREAD:
//                     Serial.println("Read too fast");
//                     break;
//                 default:
//                     Serial.println("Unknown error");
//                     break;
//             }
//             Serial.println();
//         }
//         vTaskDelay(pdMS_TO_TICKS(2000));  // Delay 2000ms
//     }
// }
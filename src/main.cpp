#include <HardwareSerial.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <DHT20.h>
#include <Wifi.h>
#include <Arduino.h>
#include <ArduinoOTA.h>

constexpr char WIFI_SSID[] = "camellia";
constexpr char WIFI_PASSWORD[] = "12344321";
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

constexpr char TOKEN[] = "1tqsqvh62gmcrj1yf0w7";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

#define SERIAL_DEBUG_BAUD 115200UL // monitor speed

// Task handles
TaskHandle_t Task0Handle = NULL;
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;
TaskHandle_t Task3Handle = NULL;
TaskHandle_t Task4Handle = NULL;

// DHT20 Sensor
DHT20 DHT;

void InitWiFi()   //start connect Wifi
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

const bool reconnect() //reconnect when wifi disconnected
{
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED)
  {
    return true;
  }
  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

//--------------- Follow instruction -----------------//
// void TaskHelloWorld(void *pvParameters)
// {
//   while (1)
//   {
//     Serial.println("Hello world");
//     vTaskDelay(pdMS_TO_TICKS(2000)); // Delay 2000ms
//   }
// }

void TaskTemp(void *pvParameters)
{
  while (1)
  {
    int status = DHT.read(); // > 1000 to not read too fast
    float temperature = DHT.getTemperature();
    Serial.print("DHT20 Temperature: ");
    Serial.print(temperature, 1);   //n,1
    Serial.println(" °C");
    tb.sendTelemetryData("temperature", temperature);  //send to thingsboard

    Serial.print("Status: "); //status of reading data
    switch (status)
    {
    case DHT20_OK:
      Serial.println("OK");
      break;
    case DHT20_ERROR_CHECKSUM: //data integrity
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
    vTaskDelay(pdMS_TO_TICKS(1500)); // Delay 1500ms  - Period
  }
}

void TaskHumi(void *pvParameters)
{
  while (1) // not read data again to avoid reading too fast
  {
    float humidity = DHT.getHumidity();
    Serial.print("DHT20 Humidity: ");
    Serial.print(humidity, 1);
    Serial.println(" %");
    tb.sendTelemetryData("humidity", humidity);

    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1000ms
  }
}

void TaskTBConnection(void *pvParameters)
{
  while (1)
  {
    if (!tb.connected())
    {
      Serial.print("Connecting to: ");
      Serial.print(THINGSBOARD_SERVER);
      Serial.print(" with token ");
      Serial.println(TOKEN);
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT))
      {
        Serial.println("Failed to connect");
      }
      vTaskDelay(pdMS_TO_TICKS(10000));  //acceptable disconnected time with TB
    }
  }
}

void TaskWifiConnection(void *pvParameters)
{
  while (1)
  {
    if (!reconnect())
    {
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C

  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();

  // Initialize DHT20
  if (!DHT.begin())
  {
    Serial.println("Failed to initialize DHT20 sensor!");
    while (1)
      ;
  }
  Serial.println("DHT20 sensor initialized.");

  // Create 
  //  xTaskCreate(TaskHelloWorld, "HelloWorld", 1000, NULL, 1, &Task0Handle);
  xTaskCreate(TaskTemp, "Temperature", 2000, NULL, 1, &Task1Handle);
  xTaskCreate(TaskHumi, "Humidity", 2000, NULL, 1, &Task2Handle);
  xTaskCreate(TaskTBConnection, "Check Connection Thingsboard", 2048, NULL, 1, &Task3Handle);
  xTaskCreate(TaskWifiConnection, "Check Wifi", 2048, NULL, 1, &Task4Handle);
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
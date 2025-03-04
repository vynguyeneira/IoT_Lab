#include <HardwareSerial.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <DHT20.h>
#include <Wifi.h>
#include <Arduino.h>
#include <ArduinoOTA.h>

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
#define LED_PIN 13
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";
constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
volatile bool attributesChanged = false;
volatile int ledMode = 0;
volatile bool ledState = false;

constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
    LED_STATE_ATTR,
    BLINKING_INTERVAL_ATTR};

RPC_Response setLedSwitchState(const RPC_Data &data)
{ // from thingsboard
  Serial.println("Received Switch state");
  bool newState = data;
  Serial.print("Switch state change: ");
  Serial.println(newState);
  digitalWrite(LED_PIN, !newState);
  attributesChanged = true;
  return RPC_Response("setLedSwitchValue", newState);
}

const std::array<RPC_Callback, 1U> callbacks = {
    RPC_Callback{"setLedSwitchValue", setLedSwitchState}};

void processSharedAttributes(const Shared_Attribute_Data &data)
{
  for (auto it = data.begin(); it != data.end(); ++it)
  {
    if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0)
    {
      const uint16_t new_interval = it->value().as<uint16_t>();
      if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX)
      {
        blinkingInterval = new_interval;
        Serial.print("Blinking interval is set to: ");
        Serial.println(new_interval);
      }
    }
    else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0)
    {
      ledState = it->value().as<bool>();
      digitalWrite(LED_PIN, ledState);
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
    }
  }
  attributesChanged = true;
}

// Task handles
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;
TaskHandle_t Task3Handle = NULL;
TaskHandle_t Task4Handle = NULL;
TaskHandle_t Task5Handle = NULL;
TaskHandle_t Task6Handle = NULL;
TaskHandle_t Task7Handle = NULL;

// DHT20 Sensor
DHT20 DHT;

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

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

const bool reconnect()
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

void TaskTemp(void *pvParameters)
{
  while (1)
  {
    int status = DHT.read(); // > 1000
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

void TaskHumi(void *pvParameters)
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

void TaskMSSV(void *pvParameters)
{
  while (1)
  {
    Serial.println("MSSV: 2012458");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Delay 2000ms
  }
}

// void TaskLEDControl(void *pvParameters)
// {
//   pinMode(GPIO_NUM_13, OUTPUT); // Initialize LED pin
//   int ledState = 0;
//   while (1)
//   {

//     if (ledState == 0)
//     {
//       digitalWrite(GPIO_NUM_13, HIGH); // Turn ON LED
//     }
//     else
//     {
//       digitalWrite(GPIO_NUM_13, LOW); // Turn OFF LED
//     }
//     ledState = 1 - ledState;
//     vTaskDelay(2000);
//   }
// }

void Task4(void *pvParameters)
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
        return;
      }

      tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

      Serial.println("Subscribing for RPC...");
      if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend()))
      {
        Serial.println("Failed to subscribe for RPC");
        return;
      }

      if (!tb.Shared_Attributes_Subscribe(attributes_callback))
      {
        Serial.println("Failed to subscribe for shared attribute updates");
        return;
      }

      Serial.println("Subscribe done");

      if (!tb.Shared_Attributes_Request(attribute_shared_request_callback))
      {
        Serial.println("Failed to request for shared attributes");
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
    if (!reconnect())
    {
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}

void Task6(void *pvParameters)
{
  while (1)
  {
    if (attributesChanged)
    {
      attributesChanged = false;
      tb.sendAttributeData(LED_STATE_ATTR, digitalRead(LED_PIN));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void Task7(void *pvParameters)
{
  while (1)
  {
    tb.loop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C

  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
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
  xTaskCreate(TaskTemp, "Temperature", 2000, NULL, 1, &Task1Handle);
  xTaskCreate(TaskHumi, "Humidity", 2000, NULL, 1, &Task2Handle);
  xTaskCreate(TaskMSSV, "MSSV", 1000, NULL, 1, &Task3Handle);
  // xTaskCreate(TaskLEDControl, "LED Control", 2048, NULL, 2, NULL);
  xTaskCreate(Task4, "Check Connection Thingsboard", 2048, NULL, 1, &Task4Handle);
  xTaskCreate(Task5, "Check Wifi", 2048, NULL, 1, &Task5Handle);
  xTaskCreate(Task6, "Send Attribute Data", 2000, NULL, 1, &Task6Handle);
  xTaskCreate(Task7, "TB Loop", 2048, NULL, 1, &Task7Handle);
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
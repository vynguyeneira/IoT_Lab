#include <HardwareSerial.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

#define SERIAL_DEBUG_BAUD 115200UL 

void setup() {
  // Initalize serial connection for debugging
  Serial.begin(SERIAL_DEBUG_BAUD);

}

void loop() {
  Serial.println("MSSV: 2012458");

  delay(1000);
}
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_BMP085.h>
#include <SPI.h>

#define LED 2
#define COOLER 12

int dutyC = 0;
char bufferDutyc[6];

float temperature = 0;
float pressure = 0;
constexpr size_t BUFFER_SIZE = 7; // 1 char for the sign, 1 char for the decimal dot, 4 chars for the value & 1 char for null termination
char bufferTemp[BUFFER_SIZE];
char bufferPres[BUFFER_SIZE];

Adafruit_BMP085 bmp;
// Wi-Fi ssid and password
const char *ssid = "ssid";
const char *password = "password";
// Your Raspberry Pi IP, in this format
IPAddress server(192, 168, 0, 22);

// Callback function, listen to the incoming message. Running in Core 1 as standard
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Incoming message: ");
  for (int i = 0; i < length; i++)
  {
    bufferDutyc[i] = (char)payload[i]; // Compose the string, char by char
  }
  dutyC = atoi(bufferDutyc); // Convert string to integer
  ledcWrite(0, dutyC);       // Set duty cycle (0-255)
  Serial.println(dutyC);
  memset(bufferDutyc, 0, 6); // Clear the buffer
}

WiFiClient esp32Client;
PubSubClient client(server, 1883, callback, esp32Client);

void startWifi()
{
  Serial.print("\nEsp32 connect with ");
  Serial.println(ssid);
  Serial.print("Starting wifi connect");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.print(" Connected. Esp32 IP address: ");
  Serial.println(WiFi.localIP());
}

void startBmp()
{
  Serial.print("Starting BMP180...");
  if (!bmp.begin())
  {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1)
    {
    }
  }
}

void publishBmp(void *parameter)
{
  Serial.println("Publish BMP180...");
  while (true)
  {
    // read sensor
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure();

    // publish temperature
    dtostrf(temperature, BUFFER_SIZE - 1 /*width*/, 1 /*precision*/, bufferTemp); // convert double to string
    client.publish("topic/temp", bufferTemp, BUFFER_SIZE);
    Serial.print(temperature);
    Serial.print(" | ");

    // publish pressure
    dtostrf(pressure, BUFFER_SIZE - 1 /*width*/, 1 /*precision*/, bufferPres);
    client.publish("topic/pres", bufferPres, BUFFER_SIZE);
    Serial.println(pressure);

    digitalWrite(LED, HIGH);
    vTaskDelay(20 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    vTaskDelay(480 / portTICK_PERIOD_MS);
  }
}

void connectBroker()
{
  while (!client.connected())
  {
    Serial.println("Attempting broker connection...");
    if (client.connect("esp32Client", "user_broker", "password_broker"))
    {
      client.publish("topic/temp", "Ready to publish Temperature in topic/temp");
      client.publish("topic/pres", "Ready to publish Pressure in topic/pres");
      client.subscribe("topic/pwm"); // Esp32 subscribe inTopic. See callback function
      Serial.println("Broker connected!");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup()
{
  // IO config
  pinMode(LED, OUTPUT);
  ledcSetup(0, 2000, 10);
  ledcAttachPin(COOLER, 0);

  Serial.begin(115200);
  startBmp();
  startWifi();
  connectBroker();

  // publish in core 0
  xTaskCreatePinnedToCore(publishBmp, "publishBmp", 10000, NULL, 1, NULL, 0);
}

void loop()
// running in Core 1 as standard
{
  // verify broker connection
  if (!client.connected())
  {
    connectBroker();
  }
  // keep broker connection
  client.loop();
}
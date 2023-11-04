#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <NewPing.h>
#include <Wire.h>
#include <MPU6050.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

const char *ssid = "vivo";
const char *password = "12345678";

const char *mqttServer = "demo.thingsboard.io"; // ThingsBoard MQTT server
const int mqttPort = 1883;
const char *accessToken = "XYvU4Vbqb1tVCeyGoFTa"; // Device access token from ThingsBoard
const char* ntpServer = "pool.ntp.org";
const long gmtOffset = 19800; // GMT+5:30 (India Standard Time)
const int daylightOffset = 0;

WiFiUDP ntpUDP;
MPU6050 mpu;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);


const int trigPin = D7; // Pin connected to the trigger pin of ultrasonic sensor
const int echoPin = D8; // Pin connected to the ultrasonic sensor's echo 
const int alarmHour = 22;   // Set alarm hour (24-hour format)
const int alarmMinute = 23;
bool alarmTriggered = false;
#define BUZZER_PIN D3 // Use the appropriate GPIO pin

NewPing sonar(trigPin, echoPin, 200);
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset, daylightOffset);


void setupWiFi()
{
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    timeClient.begin();

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.println("Message arrived [Topic: " + String(topic) + "]");
}

void reconnect()
{
    while (!mqttClient.connected())
    {
        Serial.print("Attempting MQTT connection...");
        
        if (mqttClient.connect("ArduinoClient", accessToken, NULL))
        {
            Serial.println("connected");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
        }
    }
}
/*int getTemp()
{
    sensors.requestTemperatures(); 
    float temperatureC = sensors.getTempCByIndex(0);
    //float temperatureF = sensors.getTempFByIndex(0);
    //Serial.print(temperatureC);
    //Serial.println("ºC");
    //Serial.print(temperatureF);
    //Serial.println("ºF");
    delay(1000);
    return temperatureC;
}
*/
float measureDistance()
{
  unsigned int distance = sonar.ping_cm();

  return distance;
}
void buzz_alert()
{
    tone(BUZZER_PIN, 1000); // Generate a tone of 1000Hz
    delay(1000); // Keep the tone for 1 second
    noTone(BUZZER_PIN); // Turn off the buzzer
    delay(1000); // Pause for 1 second
}
int accelerometer()
{
  // Read accelerometer and gyroscope data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  return ax;
}
int timer()
{
  timeClient.update();
  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();
  if (!alarmTriggered && currentHour == alarmHour && currentMinute == alarmMinute) {
  // Alarm condition met, trigger your action here
    Serial.println("Alarm triggered!");
    alarmTriggered = 1; // Set the alarm as triggered
  }
  for(int i = 0;i<=5;i++)
  {
    if(alarmTriggered == true)
    {
      Serial.println("Alarm Triggered");
      tone(BUZZER_PIN,1000);
      delay(1000);
    }
      return 1;
      break;
  }
    alarmTriggered = false;
    noTone(BUZZER_PIN);
    return 0;
}

void setup()
{
    Serial.begin(115200);
    setupWiFi();
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(callback);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    Wire.begin(D2,D1);
    mpu.initialize();
    Serial.println("MPU6050 initialized");
}

void loop()
{
    if (!mqttClient.connected())
    {
        reconnect();
    }
    mqttClient.loop();

    float distance = measureDistance();
    if (distance < 10)
    {
      //buzz_alert();
      Serial.println("Fuel is empty");
    }
    else if(distance > 50)
    {
      //buzz_alert();
      Serial.println("Fuel is full");
    }
    int ax = accelerometer();
    Serial.print("Distance: ");
    Serial.println(distance);
      // Print data to serial monitor
    Serial.print("Accel: ");
    Serial.print("X = "); 
    Serial.print(ax);
    if(ax > 8000)
    {
      buzz_alert();
    }
    else if(ax <= -8000)
    {
      buzz_alert();
    }
    Serial.println();
    Serial.print("Current time: ");
    Serial.println(timeClient.getFormattedTime());
    int time = timer(); 

    //String payload = "{\"distance\": " + String(distance) + "}";
    String payload = "{\"fuel\": " + String(distance) + ", \accelerometer\": " + String(ax) + ", \"timer\": " + String(time) + "}";
    char fuel[100];
    char accel[100];
    char timer[100];
    payload.toCharArray(fuel,100);
    payload.toCharArray(accel, 100);
    payload.toCharArray(timer,100);
    mqttClient.publish("v1/devices/me/telemetry",fuel);
    mqttClient.publish("v2/devices/you/telemetry",accel);
    mqttClient.publish("v3/devices/we/telemetry",timer);
    delay(1000); // Send data every 5 seconds
}

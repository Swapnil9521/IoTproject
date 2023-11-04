#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <NewPing.h>
#include <Wire.h>
#include <MPU6050.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

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
SoftwareSerial gpsSerial(D7,D8); // RX, TX pins for GPS module
TinyGPSPlus gps;

const int trigPin = D5; // Pin connected to the trigger pin of ultrasonic sensor
const int echoPin = D6; // Pin connected to the ultrasonic sensor's echo 
const int alarmHour = 16;   // Set alarm hour (24-hour format)
const int alarmMinute = 05;
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
    alarmTriggered = true; // Set the alarm as triggered
  }

  for(int i = 0; i <= 5; i++)
  {
    if (alarmTriggered)
    {
      Serial.println("Alarm Triggered");
      tone(BUZZER_PIN, 1000);
      delay(1000);
    }
    return 1;
    break;
  }

  alarmTriggered = false;
  Serial.println("Alarm Reset");
  noTone(BUZZER_PIN);
  return 0;
}

bool getGPSData(float &latitude, float &longitude)
{
    while (gpsSerial.available() > 0)
    {
        if (gps.encode(gpsSerial.read()))
        {
            if (gps.location.isValid())
            {
                latitude = gps.location.lat();
                longitude = gps.location.lng();
                return true; // GPS data successfully read
            }
        }
    }
    return false; // No valid GPS data available
}

void setup()
{
    Serial.begin(115200);
    setupWiFi();
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(callback);
    gpsSerial.begin(9600);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    Wire.begin(D2, D1);
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
    else if (distance > 50)
    {
      //buzz_alert();
      Serial.println("Fuel is full");
    }
    int ax = accelerometer();
    Serial.print("Distance: ");
    Serial.println(distance);
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
    
    float latitude, longitude;
    if (getGPSData(latitude, longitude))
    {
        Serial.print("Latitude: ");
        Serial.println(latitude, 6);
        delay(1000);
        Serial.print("Longitude: ");
        Serial.println(longitude, 6);
        delay(1000);
        String gpsPayload = "{\"latitude\": " + String(latitude) + ", \"longitude\": " + String(longitude) + "}";
        char lat[100];
        gpsPayload.toCharArray(lat,100);
        mqttClient.publish("v4/devices/gps/telemetry",lat);

    }


}

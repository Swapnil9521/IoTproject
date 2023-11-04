#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const char *ssid = "vivo";
const char *password = "12345678";

const char *mqttServer = "demo.thingsboard.io"; // ThingsBoard MQTT server
const int mqttPort = 1883;
const char *accessToken = "esp8266_ultrasonic"; // Device access token from ThingsBoard

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const int trigPin = D1; // Pin connected to the ultrasonic sensor's trigger
const int echoPin = D0; // Pin connected to the ultrasonic sensor's echo
const int oneWireBus = D2; 
const int batt_pin = D5;
#define BUZZER_PIN D3 // Use the appropriate GPIO pin
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

void setupWiFi()
{
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

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
int getTemp()
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

float measureDistance()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    float duration = pulseIn(echoPin, HIGH);
    float distance = (duration * 0.0343) / 2;

    return distance;
}
void buzz_alert()
{
    tone(BUZZER_PIN, 1000); // Generate a tone of 1000Hz
    delay(1000); // Keep the tone for 1 second
    noTone(BUZZER_PIN); // Turn off the buzzer
    delay(1000); // Pause for 1 second
}
int get_battery()
{
  int battery = digitalRead(D5);
  do
  {
    if(battery == 1)
    {
     int battery = 12;
     delay(1000);
     return battery;
    }
    else
    {
      break;
    }
  }while(battery == 1);
  return 0;
}

void setup()
{
    Serial.begin(115200);
    setupWiFi();
    mqttClient.setServer(mqttServer, mqttPort);
    mqttClient.setCallback(callback);
    sensors.begin();
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(batt_pin, INPUT);
}

void loop()
{
    if (!mqttClient.connected())
    {
        reconnect();
    }
    mqttClient.loop();

    float distance = measureDistance();
    if (distance > 30)
    {
      buzz_alert();
    }
    int temperature = getTemp();
    if (temperature > 35)
    {
      buzz_alert();
    }
    
    int battery = get_battery();
    Serial.print("Distance: ");
    Serial.println(distance);
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Battery: ");
    Serial.println(battery);

    //String payload = "{\"distance\": " + String(distance) + "}";
    String payload = "{\"distance\": " + String(distance) + ", \"temperature\": " + String(temperature) + ", \"battery\": " + String(battery) + "}";
    char attributes[100];
    char temp[100];
    char bike_batt[100];
    payload.toCharArray(temp,100);
    payload.toCharArray(attributes, 100);
    payload.toCharArray(bike_batt, 100);

    mqttClient.publish("v1/devices/me/telemetry", attributes);
    mqttClient.publish("v2/device/you/telemetry",temp);
    mqttClient.publish("v3/device/we/telemetry",bike_batt);
    delay(1000); // Send data every 5 seconds
}

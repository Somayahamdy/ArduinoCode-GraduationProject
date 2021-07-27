#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30100_PulseOximeter.h"
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <FirebaseArduino.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#define DEVICE_ID   "10002000"
////////////////////////////////////////////////////////////////////////////////////////////////////

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Function that gets current epoch time
unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
#define FIREBASE_HOST "health-care-b8f61-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "xUU37QlDAZSjJrbv9GN7Q6s0lY1Z6cjm1cGnRv9o"
#define WIFI_SSID "Osha"
#define WIFI_PASSWORD "1392016Aisha"
//
//#define WIFI_SSID "aliaa"
//#define WIFI_PASSWORD "lelelele"
//
//#define WIFI_SSID "Maryam"
//#define WIFI_PASSWORD "12345678"
//
//#define WIFI_SSID "Huda"
//#define WIFI_PASSWORD "AMIRAANDHUDA"

int errorCount = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////
float temperature;
float heartRate;
int   oxgyn;
float bloodSugre;

float Pasttemperature = -1;
float PastheartRate = -1;
int   Pastoxgyn = -1;
float PastbloodSugre = -1;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);
long temperatureTs = 0;


float getTemperature ()
{
  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);
  return tempC;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// PulseOximeter is the higher level interface to the sensor
PulseOximeter heartSensor; 
long heartTs = 0;
float convertToSugure (uint16_t ir)
{
  return ((3.3775*(ir/1000.0)) + 79.191)-96;
}
void getPulseOxi()
{

  uint16_t irRawValue,redRawValue;
  
  heartRate = heartSensor.getHeartRate();
  oxgyn = heartSensor.getSpO2();
  irRawValue = heartSensor.rawIRValue;
  redRawValue = heartSensor.rawRedValue;

  bloodSugre = convertToSugure (irRawValue);
  
  // print heart rate
  Serial.print("Heart rate:");
  Serial.print(heartRate);
  Serial.print("bpm / SpO2:");
  Serial.print(oxgyn);
  Serial.print("%");
  Serial.print("------");

  Serial.print(irRawValue);
  Serial.print("\t");
  Serial.println(redRawValue);
  Serial.println(bloodSugre);

  
}

//////////////////////////////////////////////////////////////////////////////
#define BUZZER_PIN 14
void buzzerOff()
{
  digitalWrite(BUZZER_PIN,LOW);
}
void buzzerTemperature()
{
  digitalWrite(BUZZER_PIN,HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN,LOW);
  delay(1000);
}
void buzzerHeartRate()
{
  digitalWrite(BUZZER_PIN,HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN,LOW);
  delay(2000);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Temperature and HeartRate");

  // initilize wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED)
  { Serial.print(".");  delay(500);}
  Serial.println();
  Serial.println("connected");

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);


  // initiliaze timestamp engine
  timeClient.begin();

  // initilize buzzer
  pinMode(BUZZER_PIN,OUTPUT);
  buzzerOff();
  
  // start temp sensor
  tempSensor.begin();

  // initialize heart rate sensor
  if (!heartSensor.begin()) {
        Serial.println("heartSensor FAILED");
        for(;;);
    } else {
        Serial.println("heartSensor SUCCESS");
    }


  
}


void loop() {

  if (millis()- temperatureTs >= 3000)
  {
    
    
    heartSensor.shutdown();
    
    float tempC = getTemperature();
  
    if(tempC != DEVICE_DISCONNECTED_C) 
    {
      Serial.print("Temperature for the device 1 (index 0) is: ");
      Serial.println(tempC);
      temperature = tempC;
    } 
    else
    {
      Serial.println("Error: Could not read temperature data");
    }

    temperatureTs = millis();


    


    if (temperature > 38.5)
    {
      buzzerTemperature();
    }
    else if (heartRate > 120)
    {
      buzzerHeartRate();
    }
    else
    {
      buzzerOff();
    }
    
   
      bool transmit = false;
      
              Serial.print("Abs Temp = "); Serial.print(fabs(temperature-Pasttemperature)); Serial.print("   "); Serial.print(temperature); Serial.print(","); Serial.println(Pasttemperature);

      if (temperature == DEVICE_DISCONNECTED_C && ((int)heartRate == 0 && (int)oxgyn == 0))
      {
        transmit = false;
        Serial.println("No devices Connected, Do not send");
      }
      if (fabs(temperature - Pasttemperature) > 0.2 || fabs(heartRate- PastheartRate) > 10 || fabs (oxgyn - Pastoxgyn) > 1 || fabs(bloodSugre - PastbloodSugre) > 3)
      {
        transmit = true;
      }
      if (Pasttemperature <0 && PastheartRate < 0 && Pastoxgyn < 0 && PastbloodSugre < 0)
      {
        transmit = true;
        Serial.println("First Reading");
      }

      if (transmit == true)
      {
        int count = Firebase.getInt("Readings/count");
        if (Firebase.failed()) {
            Serial.print("Reading Count failed:");
            Serial.println(Firebase.error());  
            errorCount++;
            if (errorCount == 50)
            {
              ESP.reset();
            }
        }
        else
        {
          errorCount=0;
        
        count++;
        Firebase.setInt("Readings/count",count);
    
        String path;
        path = "Readings/" + String(count);
    
        String temp_path = path + "/temp";
        String heart_path = path + "/heart";
        String oxgyn_path = path + "/oxgyn";
        String blood_path = path + "/blood";
        String device_path = path + "/device_id";
        String timestamp_path = path + "/timestamp";
    
        unsigned long timestamp = getTime();
    
        Firebase.set(temp_path,temperature);
        Firebase.set(heart_path,heartRate);
        Firebase.set(oxgyn_path,oxgyn);
        Firebase.set(blood_path,bloodSugre);
        Firebase.set(device_path,DEVICE_ID);
        Firebase.set(timestamp_path,timestamp);

        Pasttemperature = temperature;
        Pastoxgyn = oxgyn;
        PastheartRate = heartRate;
        PastbloodSugre = bloodSugre;
      }
      }

      
    

    heartSensor.resume();
  }
  


  // Make sure to call update as fast as possible
  heartSensor.update();

  if (millis() - heartTs >= 1000)
  {
    getPulseOxi();
    heartTs = millis();
  }


}

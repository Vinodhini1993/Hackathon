#  IoT Based Smart Assistive Device for Pain Detection and Management

##  Introduction
<p align="justify">
People with disabilities benefit greatly from prosthetic legs, which enable them restore their freedom and movement in day-to-day activities. An essential part of these prosthetic limbs is the liner, which is typically composed of silicone and acts as a barrier between the user's residual limb and the artificial leg. But because the liner rubs against the skin, a person may feel uncomfortable when they first start wearing a prosthetic leg. The bones and muscles of the residual limb may become irritated and under more pressure as a result of this friction. The user may find it difficult to use the prosthetic comfortably for prolonged periods of time as a result of the intense pain that this pressure can eventually produce. The user may need to experiment with various liners or accessories as they get used to the prosthetic leg.


##  Overview
<p align="justify">
The aim of this system is to evaluate the adaptability of assistive devices, particularly prosthetic legs, to improve the comfort and functionality for the user. By providing real-time data access, this system enhances the accessibility and user experience of the prosthetic leg. It works in collaboration with an occupational therapist to increase the durability of the device and includes a pain monitoring system that tracks and assesses the discomfort caused by wearing the prosthetic leg. Through continuous real-time data monitoring and feedback, the system enables adaptive pain management, offering personalized adjustments to minimize discomfort. The collected data is analyzed and visualized, giving both the user and healthcare professionals valuable insights to ensure a more comfortable and effective prosthetic experience. This approach helps improve the overall quality of life for prosthetic leg users, addressing both physical and emotional needs.

## Features and benefits of the product

Real-time data monitoring and feedback

Adaptive Pain Management for User

Data Analysis and Visualization

## Advantage of the product

To check the adaptability of the assistive device

Provide real-time data access and enhances accessibility

Increase durability of the assistive device with occupational therapist, providing a pain monitoring system about effect of pain caused by the prosthetic leg.

The primary beneficiaries of this approach are locomotors differently abled or Person with Disability (PwD). People of all ages reduce their discomfort and improve the efficiency of prosthetic leg.


![3](https://github.com/user-attachments/assets/ef9da4e9-8324-4e73-9b76-424044960926)


## Components required with Bill of Materials
| Item                   | Quantity | Description                                                   | Links to Products                                      |
|------------------------|----------|---------------------------------------------------------------|---------------------------------------------------|
| VSD Squadron Mini      | 1        | Microcontroller board                                         | https://sulkurl.com/kR9                           |
| ESP8266(Wifi)          | 1        | Wifi Module                                                   |                                                   |
| EMG Sensor             | 1        | Sensor module for muscle activity                             | https://sulkurl.com/kRZ                           |
| BME Sensor             | 1        | Sensor module for skin temperature                            | https://sulkurl.com/kR1                           |

## Table for Pin Connections

|  S.no         | Muscle BioAmp Candy Sensor | VSD Squadron Mini Board   |                                             
|---------------|----------|---------------------------------------------------------------|
| 1.            |  VCC     |   VDD(5V) |
| 2.            |  GND     |   VSS     |
| 3.            |  OUT     |   PA2     |

|  S.no         | ESP8266(Wifi Module) | VSD Squadron Mini Board * and Cloud   |                                             
|---------------|----------|---------------------------------------------------------------|
| 1.            | D4       |  PD0*  |
| 2.            | Rx       | Thingspeak(Cloud Connection) | 

|  S.no         |GY-BME280-5V Temperature Sensor  | VSD Squadron Mini Board   |                                             
|---------------|----------|---------------------------------------------------------------|
| 1.            | VIN      | VDD |
| 2.            | GND      | VSS |
| 3.            | SCL      | PC2 |
| 4.            | SDA      | PC1 |

##  Pinout Diagram


![2](https://github.com/user-attachments/assets/f26f1e46-5980-4a49-a953-8b9eaa4dc1f0)


## Source Code
```
#include <Adafruit_BMP280.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include "ThingSpeak.h"
#include <Wire.h>

#define SAMPLE_RATE 500
#define BAUD_RATE 115200
#define INPUT_PIN A0
#define BUFFER_SIZE 128

int circular_buffer[BUFFER_SIZE];
int data_index, sum;

const char* ssid = "Break Your Leg";
const char* password = "mafq6071" ;
//const char* server = "api.thingspeak.com";
//String apiKey = "U9ONPVWLY2D0OHL9"; // Replace with your ThingSpeak Write API Key
const char * WriteAPIKey = "F5GMH511YMW8ED76"; // Your write API Key

Adafruit_BMP280 bmp; // I2C Interface

unsigned long Channel_ID = 2822635;
WiFiClient  client;

void setup() {
  Serial.begin(115200);
   //delay(10);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  //delay(10);
  WiFi.begin(ssid, password);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    //delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("NodeMcu connected to wifi...");
  Serial.println(ssid);
  
  ThingSpeak.begin(client);
  
}

void loop(){
  emg();
 bmp_temp();
}


void bmp_temp() {
  Serial.println(F("BMP280 test"));
  //if (!bmp.begin()) {
  if (!bmp.begin(0x76,0x58)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    Serial.println();
    //delay(1);
    ThingSpeak.writeField(Channel_ID, 1,bmp.readTemperature(), WriteAPIKey);
}
void emg() {
   //Serial.begin(BAUD_RATE);
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;

  // Run timer
  static long timer = 0;
  timer -= interval;

  // Sample and get envelop
  if(timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    int sensor_value = analogRead(INPUT_PIN);
    int signal = EMGFilter(sensor_value);
    int envelop = getEnvelop(abs(signal));
    Serial.print(signal);
    Serial.print(",");
    Serial.println(envelop);
    //delay(1);
  }
}

// Envelop detection algorithm
int getEnvelop(int abs_emg){
  sum -= circular_buffer[data_index];
  sum += abs_emg;
  circular_buffer[data_index] = abs_emg;
  data_index = (data_index + 1) % BUFFER_SIZE;
  return (sum/BUFFER_SIZE) * 2;
}

// Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
// Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
// Filter is order 4, implemented as second-order sections (biquads).
// Reference: 
// https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
// https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
float EMGFilter(float input)
{
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}
```

## Demo Video

https://drive.google.com/file/d/1MX8_-SOmx8ajoxNuYKaFquLBDQpJgDpC/view?usp=sharing

https://drive.google.com/file/d/1flbyY-fuYytsHpno1tzOKj0mSjZX8Qn6/view?usp=sharing

## Conclusion
<p align="justify">
This project presents a smart IoT-based assistive device designed to monitor pain indicators such as muscle tension and body temperature in real-time, enabling users to manage pain
effectively and maintain greater independence. The prosthetic leg is equipped with a
(silicone) liner to reduce friction and pressure, alleviating discomfort and pain from extended
use. By conducting durability and pain threshold tests, ensuring long-term comfort and
usability, ultimately improving the quality of life for users by making their daily activities
more manageable.

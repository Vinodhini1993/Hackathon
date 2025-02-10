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


## Source Code (Arduino IDE)
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

## Source Code (VSDSquadron Mini )
```
// Defining the SDA and SCL Pins for I2C Communication
#include <debug.h>
#include <ch32v00x.h>
#include <ch32v00x_gpio.h>

// Defining the SDA and SCL Pins for I2C Communication
#define SDA_PIN GPIO_Pin_1
#define SCL_PIN GPIO_Pin_2
//#define SDA_PIN GPIO_Pin_3
//#define SCL_PIN GPIO_Pin_4

// Defining the LCD Address
#define LCD_Address 0x27

// BME280 I2C Address
#define BME280_Address 0x77
#define I2C_PORT GPIOC
// BME280 Calibration Coefficients
//int16_t ac1, ac2, ac3;
//uint16_t ac4, ac5, ac6;
//int16_t b1, b2;
//int16_t mb, mc, md;

void lcd_send_cmd(unsigned char cmd);
void lcd_send_data(unsigned char data);
void lcd_send_str(unsigned char *str);
void lcd_init(void);
void delay_ms(unsigned int ms);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(unsigned char dat);
void i2c_ACK(void);
//void bme280_read_calibration_data(void);
int16_t bme280_read_temperature(void);
//int32_t bme280_read_pressure(void);
void bme280_write_register(unsigned char reg, unsigned char value);
unsigned char bme280_read_register(unsigned char reg);
uint16_t bme280_read_16bit_register(unsigned char reg);

// Function to produce a delay
void delay_ms(unsigned int ms) {
    for (unsigned int i = 0; i < ms; i++) {
        for (unsigned int j = 0; j < 8000; j++) {
            __NOP();
        }
    }
}

// Function to initialize GPIO pins
void GPIO_INIT(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // Initialize SDA and SCL pins for I2C
    GPIO_InitStructure.GPIO_Pin = SDA_PIN | SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// Function to write a byte of data to the I2C bus
void i2c_write(unsigned char dat) {
    for (unsigned char i = 0; i < 8; i++) {
        GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
        if (dat & (0x80 >> i)) {
            GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
        } else {
            GPIO_WriteBit(GPIOC, SDA_PIN, Bit_RESET);
        }
        GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    }
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
}

// Function to start I2C communication
void i2c_start(void) {
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_RESET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
}

// Function to stop I2C communication
void i2c_stop(void) {
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_RESET);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
}

// Function to wait for an acknowledgment bit
void i2c_ACK(void) {
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    while(GPIO_ReadInputDataBit(GPIOC, SDA_PIN));
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
}

// Function to send a command to the LCD
void lcd_send_cmd(unsigned char cmd) {
    unsigned char cmd_l = (cmd << 4) & 0xf0;
    unsigned char cmd_u = cmd & 0xf0;

    i2c_start();
    i2c_write(LCD_Address << 1);
    i2c_ACK();
    i2c_write(cmd_u | 0x0C);
    i2c_ACK();
    i2c_write(cmd_u | 0x08);
    i2c_ACK();
    delay_ms(1);
    i2c_write(cmd_l | 0x0C);
    i2c_ACK();
    i2c_write(cmd_l | 0x08);
    i2c_ACK();
    delay_ms(1);
    i2c_stop();
}

// Function to send data to the LCD
void lcd_send_data(unsigned char data) {
    unsigned char data_l = (data << 4) & 0xf0;
    unsigned char data_u = data & 0xf0;

    i2c_start();
    i2c_write(LCD_Address << 1);
    i2c_ACK();
    i2c_write(data_u | 0x0D);
    i2c_ACK();
    i2c_write(data_u | 0x09);
    i2c_ACK();
    delay_ms(1);
    i2c_write(data_l | 0x0D);
    i2c_ACK();
    i2c_write(data_l | 0x09);
    i2c_ACK();
    delay_ms(1);
    i2c_stop();
}

// Function to send a string to the LCD
void lcd_send_str(unsigned char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

// Function to initialize the LCD
void lcd_init(void) {
    lcd_send_cmd(0x02); // Return home
    lcd_send_cmd(0x28); // 4-bit mode, 2 lines, 5x7 dots
    lcd_send_cmd(0x0C); // Display On, cursor off
    lcd_send_cmd(0x06); // Increment cursor (shift cursor to right)
    lcd_send_cmd(0x01); // Clear display
    delay_ms(20);       // Wait for the LCD to process the clear command
}

// Function to write a value to a BME280 register
void bme280_write_register(unsigned char reg, unsigned char value) {
    i2c_start();
    i2c_write(BME280_Address << 1);
    i2c_ACK();
    i2c_write(reg);
    i2c_ACK();
    i2c_write(value);
    i2c_ACK();
    i2c_stop();
}

// Function to read a value from a BME280 register
unsigned char bme280_read_register(unsigned char reg) {
    unsigned char value;
    i2c_start();
    i2c_write(BME280_Address << 1);
    i2c_ACK();
    i2c_write(reg);
    i2c_ACK();
    i2c_start();
    i2c_write((BME280_Address << 1) | 0x01);
    i2c_ACK();
    value = GPIO_ReadInputDataBit(GPIOC, SDA_PIN);
    i2c_stop();
    return value;
}

// Function to read a 16-bit value from a BME280 register
uint16_t bme280_read_16bit_register(unsigned char reg) {
    uint16_t value;
    i2c_start();
    i2c_write(BME280_Address << 1);
    i2c_ACK();
    i2c_write(reg);
    i2c_ACK();
    i2c_start();
    i2c_write((BME280_Address << 1) | 0x01);
    i2c_ACK();
    value = (bme280_read_register(reg) << 8) | bme280_read_register(reg + 1);
    i2c_stop();
    return value;
}
    
/*
// Function to read calibration data from BMP180
void bmp180_read_calibration_data(void) {
    ac1 = bmp180_read_16bit_register(0xAA);
    ac2 = bmp180_read_16bit_register(0xAC);
    ac3 = bmp180_read_16bit_register(0xAE);
    ac4 = bmp180_read_16bit_register(0xB0);
    ac5 = bmp180_read_16bit_register(0xB2);
    ac6 = bmp180_read_16bit_register(0xB4);
    b1 = bmp180_read_16bit_register(0xB6);
    b2 = bmp180_read_16bit_register(0xB8);
    mb = bmp180_read_16bit_register(0xBA);
    mc = bmp180_read_16bit_register(0xBC);
    md = bmp180_read_16bit_register(0xBE);
}
*/
// Function to read temperature from BME280
int16_t bme280_read_temperature(void) {
    int16_t ut;
    bme280_write_register(0xF4, 0x2E);
    delay_ms(5);
    ut = bme280_read_16bit_register(0xF6);
    return ut;
}
/*
// Function to read pressure from BME280
int32_t bme280_read_pressure(void) {
    int32_t up;
    bme280_write_register(0xF4, 0x34 + (3 << 6));
    delay_ms(5);
    up = bme280_read_16bit_register(0xF6);
    return up;
}
*/
int main(void) {
    GPIO_INIT(); // Initialize GPIO pins
    lcd_init();  // Initialize the LCD Display
    delay_ms(20);
    
    // Read BMP180 calibration data
    //bmp180_read_calibration_data();
    
    // Print "Hello World" on the LCD 
    while (1) {
       int16_t temperature = bme280_read_temperature();
        //int32_t pressure = bme280_read_pressure();
        
        lcd_send_cmd(0x80); // Move the cursor to first row, first column
        delay_ms(20);
        lcd_send_str((unsigned char*)"Temp: ");
        lcd_send_data(temperature) ;
        //lcd_send_data((temperature % 10) + '0');
        lcd_send_str((unsigned char*)" C");
        
        //lcd_send_cmd(0x80); // Move the cursor to first row, first column
        //delay_ms(20);
        //lcd_send_str((unsigned char*)"Temp: ");
        //lcd_send_cmd(0xC0); // Move the cursor to second row, first column
        //lcd_send_str((unsigned char*)"Status");
        //lcd_send_data((pressure));
       //lcd_send_data((pressure / 100) + '0');
        //lcd_send_data(((pressure % 100) / 10) + '0');
        //lcd_send_data((pressure % 10) + '0');
        //lcd_send_str((unsigned char*)" hPa");
        
        //delay_ms(1000);
        //lcd_send_cmd(0x01); // Clear the display
       // delay_ms(1000);
    }
}

```



## Demonstration Video of Arduino IDE

https://drive.google.com/file/d/1MX8_-SOmx8ajoxNuYKaFquLBDQpJgDpC/view?usp=sharing

## Demonstration Video of VSDSquadron Mini 

https://drive.google.com/file/d/1flbyY-fuYytsHpno1tzOKj0mSjZX8Qn6/view?usp=sharing

## Conclusion
<p align="justify">
This project presents a smart IoT-based assistive device designed to monitor pain indicators such as muscle tension and body temperature in real-time, enabling users to manage pain
effectively and maintain greater independence. The prosthetic leg is equipped with a
(silicone) liner to reduce friction and pressure, alleviating discomfort and pain from extended
use. By conducting durability and pain threshold tests, ensuring long-term comfort and
usability, ultimately improving the quality of life for users by making their daily activities
more manageable.

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
#include <ch32v00x.h>
#include <ch32v00x_gpio.h>
#include <stdio.h> // Include stdio.h for sprintf

// Defining the SDA and SCL Pins for I2C Communication
#define SDA_PIN GPIO_Pin_1
#define SCL_PIN GPIO_Pin_2

// Defining the LCD_Address 
#define LCD_Address 0x27

const unsigned char row_offsets[] = {0x00, 0x40}; // Row 1 = 0x00, Row 2 = 0x40

void lcd_send_cmd(unsigned char cmd);
void lcd_send_data(unsigned char data);
void lcd_send_str(signed char *str);
void lcd_init(void);
void delay_ms(unsigned int ms);

// Function to produce a delay
void delay_ms(unsigned int ms) {
    for (unsigned int i = 0; i < ms; i++) {
        for (unsigned int j = 0; j < 8000; j++) {
            __NOP();
        }
    }
}

void USARTx_CFG(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1, ENABLE);

    /* USART1 TX-->D.5   RX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

// Function to initialize GPIO pins
void GPIO_INIT(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC, ENABLE);

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

void lcd_set_cursor(unsigned char row, unsigned char col) {
    if (row > 1) row = 1;  // Ensure row is within valid range (0 or 1)
    lcd_send_cmd(0x80 | (row_offsets[row] + col));
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
void lcd_send_str(signed char *str) {
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
    delay_ms(20); // Wait for the LCD to process the clear command
}

// Function to set pin mode dynamically
void set_pin_mode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef GPIO_Mode) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void receive() {
    char buffer[20]; // Buffer to store received data
    uint8_t index = 0;
    int temperature = 0, emg_val = 0, emg_env = 0;
    uint8_t startParsing = 0; // Flag to indicate when to start storing data

    while (1) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); // Wait for data

        char receivedChar = USART_ReceiveData(USART1); // Read received character

        if (receivedChar == 'x') { 
            startParsing = 1; // Start storing data once 'x' is found
            index = 0; // Reset index to store only meaningful data
        }
        
        if (receivedChar == 'o') { // End of data (carriage return)
            buffer[index] = '\0'; // Null-terminate the string
            break;
        }

        if (startParsing) { // Store only after 'x' is found
            if (index < sizeof(buffer) - 1) {
                buffer[index++] = receivedChar;
            }
        }
    }

    // Extract values from the cleaned buffer (Expected format: x30y59z80)
    sscanf(buffer, "x%d y%d z%d", &temperature, &emg_val, &emg_env);

    // lcd_send_cmd(0x01); // Clear display
    // delay_ms(20);

    // // Display Temperature
    // lcd_send_cmd(0x80); // Move cursor to first row, first column
    // lcd_send_str((signed char *)"****Scanning****");
    // delay_ms(1000);

    // // Display Temperature
    // lcd_send_cmd(0x01);
    // delay_ms(20);
    // lcd_send_cmd(0x80); // Move cursor to first row, first column
    // lcd_send_str((signed char *)"Temperature: ");
    // char tempStr[5];
    // sprintf(tempStr, "%d", temperature);
    // lcd_send_str((signed char *)tempStr);
    // delay_ms(1000);

    // // Display Pressure
    // lcd_send_cmd(0x01);
    // delay_ms(20);
    // lcd_send_cmd(0x80); // Move cursor to second row, first column
    // lcd_send_str((signed char *)"EMG_Val: ");
    // char emg_val_Str[5];
    // sprintf(emg_val_Str, "%d", emg_val);
    // lcd_send_str((signed char *)emg_val_Str);

    lcd_send_cmd(0x01); // Clear display
    //delay_ms(20);

    // Display Temperature
    lcd_set_cursor(1, 0);
    lcd_send_str((signed char *)"T: ");
    char tempStr[5];
    sprintf(tempStr, "%d", temperature);
    lcd_send_str((signed char *)tempStr);

    // Display EMG Value
    lcd_set_cursor(1, 6);
    lcd_send_str((signed char *)"EMG: ");
    char emg_val_Str[5];
    sprintf(emg_val_Str, "%d", emg_val);
    lcd_send_str((signed char *)emg_val_Str);

    // Display Result
    if (emg_val <= 120){
        lcd_set_cursor(0, 0);
        lcd_send_str((signed char *)"****Scanning****");
    }
    else if ((emg_val > 120) && (emg_val <= 150)) {
        lcd_set_cursor(0, 0);
        lcd_send_str((signed char *)"     Nominal    ");
        delay_ms(500);
    } else if ((emg_val > 150) && (emg_val <= 190)) {
        lcd_set_cursor(0, 0);
        lcd_send_str((signed char *)"    Bearable    ");
        delay_ms(500);
    }
    else if (emg_val > 190) {
        lcd_set_cursor(0, 0);
        lcd_send_str((signed char *)"  Not Bearable  ");
        delay_ms(500);
    }
}

int main(void) {
    SystemCoreClockUpdate();
    USARTx_CFG();

    GPIO_INIT(); // Initialize the GPIO pins
    delay_ms(20);

    // Initialize the LCD Display
    lcd_init();
    delay_ms(20);

    while (1) {
        receive();
    }
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

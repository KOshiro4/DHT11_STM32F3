# DHT11_STM32F3
## Description:
This code is a platform specific (STM32F3) low-level driver that enables the use of the DHT11
temperature and humidity sensor. The driver is written entirely in C using direct register
manipulation. Data gathering follows the proprietary communication protocol specified in the
DHT11 datasheet. Error handling in the forms of a checksum verification and a timeout if the
sensor is outputting strange data. More documentation can be found in the files.
## Problem Statement:
The temperature in Seattle has been noticeably lower this past month, and I want to track the
temperature changes throughout the day as well as compare temperatures across different days
of the week over time. To achieve this, I plan to use a readily available DHT11 sensor. Since the
DHT11 doesn’t use I2C or SPI, I’m limited to a proprietary communication protocol. As a result,
I’ll need to create my own data-gathering program to interface with the sensor.


/**
 * @brief Basic driver that allows the use of a single DHT11 temperature and humidity sensor with the stm32f3k0k8 and similar MCU's
 * data sheet:https://components101.com/sites/default/files/component_datasheet/DFR0067%20DHT11%20Datasheet.pdf
 * Some parts inspired by the Arduino DHT-sensor-library https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp#L362
 *
 * @IMPORTANT Make sure to enable clock access to GPIO port BEFORE trying to initialize dht11. (RCC->AHBENR)
 * @IMPORTANT Enable Timer 2 in default 32 bit counter mode
 * @IMPORTANT CLK_SPEED definition needs to be set to the clock speed of MCU
 */

#include <stdio.h>
#include <stdint.h>
#include "stm32f3xx.h"
#include "dht11.h"

//Constants
#define CLK_SPEED 16000000 //MHz
#define TIMEOUT UINT32_MAX
#define LOW 0
#define HIGH 1



// enable clock access for gpio port before using

/**
 * @brief Initialize a DHT11 sensor instance.
 * @param gpio Pointer to the GPIO port.
 * @param dataPin The pin number used for data communication.
 * @param dataArray Pointer to an int array to store the sensor data.
 * @param size of dataArray to make sure its the right size
 * @return Pointer to the initialized dht11 structure or NULL on failure.
 * @IMPORTANT Make sure to enable clock access to GPIO port BEFORE trying to initialize dht11. (RCC->AHBENR)
 * @IMPORTANT Enable Timer 2 in default 32 bit counter mode
 * @IMPORTANT CLK_SPEED definition needs to be set to the clock speed of MCU
 * @IMPORTANT array must be of size 5
 */
dht11* dht11_init(GPIO_TypeDef *gpio, int dataPin, uint8_t *dataArray, size_t arraySize) {
	static dht11 sensor = {0}; // init struct

	// check if valid array
	if(dataArray == NULL || (arraySize != 5)) {
		return NULL;
	}


	//assign
	sensor.gpio = gpio;
	sensor.dataPin = dataPin;
	sensor.data = dataArray;


	return &sensor;
}




 /**
  * @brief Retrieve data from the DHT11 sensor.
  * @param sensor Pointer to the dht11 structure.
  * @return 0 on success, non-zero on error.
  * @notes delay timings are from the data sheet
  * @notes Can't sample faster than once every two seconds per data sheet
  */
int get_data(dht11 *sensor) {
	uint32_t prim;


	// clear old data
	for(int i = 0; i < 5; i++) {
			sensor->data[i] = 0;
	}


	pinClearGPIO(sensor);
	pinPullDownPUPDR(sensor); //enable internal pull down resistor
	pinPullUpPUPDR(sensor); // enable internal pull up resistor
	DelayUS(1000); // 1ms


	pinClearGPIO(sensor);
	sensor->gpio->MODER |= (0b01 << (2 * sensor->dataPin)); // set GPIO pin as an output
	pinPullDownPUPDR(sensor);
	DelayUS(20000); // wait for 18 ms per data sheet. extra 2 ms for safety


	pinPullUpPUPDR(sensor);
	DelayUS(45); // waiting for sensor response signal


	pinClearGPIO(sensor);


	sensor->gpio->PUPDR &= ~(0b11 << (2 * sensor->dataPin)); // reset pins to normal input mode so no pull up or down

	// From Gabriel Staples on Stack overflow https://stackoverflow.com/a/71626598
	// disables interrupts for time critical task
	prim = __get_PRIMASK(); // retains interrupt status
	__disable_irq();


	// waiting for sensor to pull high. first pulls low for ~80us
	expectPulse(LOW, sensor);


	uint32_t cycles[80] = {0}; // 40 bits of data each with a low and high pulse

	// wait until it pulls down. starts data transmission by going low.
	expectPulse(HIGH, sensor);


	// used to store data in alternating fashion
	for (int i = 0; i < 80; i+=2) {
		cycles[i] = expectPulse(LOW, sensor);
		cycles[i+1] = expectPulse(HIGH, sensor);
	}


	// data collection is over. interrupts can come back
	if (!prim) {
		__enable_irq();
	}


	// loop used to determine if sensor data is a 1 or a 0 and to reflect that in the data array
	for(int i = 0; i < 40; ++i) {
		uint32_t lowCycles = cycles[2*i];
		uint32_t highCycles = cycles[2*i + 1];


		if((lowCycles == TIMEOUT) || (highCycles == TIMEOUT)) {
			return 10000; // return very large number if error happens with data collection
		}


		sensor->data[i/8] <<= 1;


		if(highCycles > lowCycles) { // means that more time was spent HIGH than LOW meaning a 1
			sensor->data[i/8] |= 1;
		}
	}


	uint8_t dhtData[5];

	// could be done without copying data but done for clarity. Max sample time slow(0.5Hz)
	for (int i = 0; i < 5; i++) {
	    dhtData[i] = sensor->data[i];
	}

	int checksum =  (dhtData[3] + dhtData[2] + dhtData[1] + dhtData[0]) & 0xFF;

	if(dhtData[4] == checksum) {

		return 0; // data matches

	} else {

		return -1; // data doesn't match

	}
	return 0;
}



/**
 * @brief Read Temp from the DHT11 Sensor
 * @param sensor Pointer to dht11 structure
 * @return Temperature in degrees Fahrenheit
 */
int readTemp(dht11 *sensor) {
	int f = sensor->data[2]; // integer portion


	if(sensor->data[3] & 0x80) { // if negative
		f = -1 - f;
	}

	return f * 1.8 + 32;
}



/**
 * @brief Read Humidity from the DHT11 Sensor
 * @param sensor Pointer to dht11 structure
 * @return Humidity in Relative Humidity percentage
 */
int readHumidity(dht11 *sensor) {
	int h = sensor->data[0]; // integer portion
	return h;
}



/**
 * @brief Clears the mode of the GPIO pin to be an input
 * @param sensor Pointer to dht11 structure
 */
void pinClearGPIO(dht11 *sensor) {
	sensor->gpio->MODER &= ~(0b11 << (2 * sensor->dataPin)); // sets pin to be a input
}



/**
 * @brief enables pull down resistor on the GPIO pin
 * @param sensor Pointer to dht11 structure
 */
void pinPullDownPUPDR(dht11 *sensor) {
	sensor->gpio->PUPDR = (sensor->gpio->PUPDR & ~(0b11 << (2 * sensor->dataPin))) | (0b10 << (2 * sensor->dataPin));

}



/**
 * @brief enables pull resistor on the GPIO pin
 * @param sensor Pointer to dht11 structure
 */
void pinPullUpPUPDR (dht11 *sensor) {
	sensor->gpio->PUPDR = (sensor->gpio->PUPDR & ~(0b11 << (2 * sensor->dataPin))) | (0b01 << (2 * sensor->dataPin));
}



// from st forums author: TDK link: https://community.st.com/t5/stm32-mcus-products/stm32g030-microsecond-delay/m-p/620290/highlight/true#M230217
/**
 * @brief creates a non blocking delay in microseconds
 * @param A number in uS desired to delay by
 */
void DelayUS(uint32_t us) {
    uint32_t start = TIM2->CNT; // highest res timer on stm32f3k0k8
    uint32_t duration = us * 16; // 16mhz timer
    while (TIM2->CNT - start < duration);
}



// inspired by DHT-sensor-library https://github.com/adafruit/DHT-sensor-library/blob/master/DHT.cpp#L362
/**
 * @brief counts the amount of clock cycles to change logic levels
 * @param either 0/1 indicating expected logic level at the current time
 * @param sensor Pointer to dht11 structure
 * @return The amount of clock cycles taken to change logic levels
 * @notes if time taken to change is >1ms a large value is returned
 */
uint32_t expectPulse(int level, dht11 *sensor) {
	// if not logic values (0,1)
	if((level < 0) || (level > 1)) {
		return TIMEOUT;
	}


	uint16_t count = 0;
	uint16_t clockCycles_ms = CLK_SPEED / 1000; // # of clock cycles per 1 ms


	if(level == LOW) {
		// wait for the pin to go high
		while(!(sensor->gpio->IDR & (1<<sensor->dataPin))){

			if(count++ >= clockCycles_ms) {
				// if >1ms worth of cycles
				return TIMEOUT;

			}
		}
	} else {
		// wait for pin to go low
		while(sensor->gpio->IDR & (1<<sensor->dataPin)){

			if(count++ >= clockCycles_ms) {
				// if >1ms worth of cycles
				return TIMEOUT;

			}
		}
	}

	return count;

}

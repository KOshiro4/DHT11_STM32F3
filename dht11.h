#ifndef DHT11_H
#define DHT11_H


#include "stm32f3xx.h" // stm32f3 header files


// DHT11 sensor Structure
typedef struct dht11 {
    GPIO_TypeDef *gpio;
    int dataPin;
    uint8_t *data;
} dht11;



dht11* dht11_init(GPIO_TypeDef *gpio, int dataPin, uint8_t *dataArray, size_t arraySize);
int get_data(dht11 *sensor);
void DelayUS(uint32_t us);
void pinClearGPIO(dht11 *sensor);
void pinPullDownPUPDR(dht11 *sensor);
void pinPullUpPUPDR (dht11 *sensor);
uint32_t expectPulse(int level, dht11 *sensor);
int readTemp(dht11 *sensor);
int readHumidity(dht11 *sensor);


#endif // DHT11_H

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported constants --------------------------------------------------------*/
#define MUX_GPIOX GPIOA
#define MUX_A0 GPIO_PIN_1
#define MUX_A1 GPIO_PIN_2
#define MUX_A2 GPIO_PIN_3

#define PGA_CS_GPIOX GPIOB
#define PGA_CS GPIO_PIN_0
#define ADC_CS_GPIOX GPIOB
#define ADC_CS GPIO_PIN_1


/* Exported function prototypes ----------------------------------------------*/
void MUX_select_channel(uint8_t channel);
void PGA_set_gain(uint8_t gain, SPI_HandleTypeDef *hspi);
uint16_t *get_samples_rev02(uint16_t noSamples, uint8_t *channels, uint8_t noChannels, SPI_HandleTypeDef *hspi);


#endif

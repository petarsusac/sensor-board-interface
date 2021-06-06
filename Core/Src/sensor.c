#include "sensor.h"
#include <stdlib.h>


/**
  * @brief  Selects MUX channel by channel label.
  *
  * @param  channel Channel label (1-8)
  *
  * @retval None
  */
void MUX_select_channel(uint8_t channel)
{

	if(channel > 8 || channel < 1) return;

	channel -= 1; // mux channels are labeled 1-8, but corresponding logical values are 0-7

	uint8_t bit;

	bit = channel & 0x01;
	HAL_GPIO_WritePin(MUX_GPIOX, MUX_A0, bit);

	bit = (channel & 0x02) >> 1;
	HAL_GPIO_WritePin(MUX_GPIOX, MUX_A1, bit);

	bit = (channel & 0x04) >> 2;
	HAL_GPIO_WritePin(MUX_GPIOX, MUX_A2, bit);
}


/**
  * @brief  Sets PGA gain.
  *
  * @param	hspi pointer to a SPI_HandleTypeDef structure that contains the configuration
  * 		information for SPI module.
  * @param  gain PGA gain. Must be 1, 2, 4, 5, 8, 10, 16 or 32. All other values are ignored
  * 		and no action is taken.
  *
  * @retval None
  */
void PGA_set_gain(SPI_HandleTypeDef *hspi, uint8_t gain)
{
	uint8_t gainValue = 0;

	switch (gain) {
		case 1:
			gainValue = 0x00;
			break;
		case 2:
			gainValue = 0x01;
			break;
		case 4:
			gainValue = 0x02;
			break;
		case 5:
			gainValue = 0x03;
			break;
		case 8:
			gainValue = 0x04;
			break;
		case 10:
			gainValue = 0x05;
			break;
		case 16:
			gainValue = 0x06;
			break;
		case 32:
			gainValue = 0x07;
			break;
		default:
			// invalid value. Ignore and return.
			return;
	}

	// MSB - command word (40 - write to register), LSB - gain value
	uint16_t spiOutputBuffer = 0x4000 | gainValue;

	HAL_GPIO_WritePin(PGA_CS_GPIOX, PGA_CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, (uint8_t *) &spiOutputBuffer, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(PGA_CS_GPIOX, PGA_CS, GPIO_PIN_SET);
}


/**
  * @brief  Receives the desired number of samples from ADC via SPI. Works for Rev02 Sensor Board.
  *
  * @note	Stores samples in a two-dimensional array of 16-bit unsigned integers, where rows represent
  * 		samples, and columns represent channels. Due to a large delay needed to stabilize ADC output
  * 		when switching between channels, samples are taken by columns (e.g. all samples from channel
  * 		1 are taken before moving on to channel 2).
  *
  * @param	hspi Pointer to a SPI_HandleTypeDef structure that contains the configuration
  * 		information for SPI module.
  * @param  noSamples Number of samples to be taken.
  * @param	channels Pointer to an array containing labels of channels to be sampled (1-8).
  * @param	noChannels Number of channels.
  *
  * @retval Pointer to an array containing samples.
  */
uint16_t *get_samples_rev02(SPI_HandleTypeDef *hspi, uint16_t noSamples, uint8_t *channels, uint8_t noChannels)
{
	uint16_t spiInputBuffer;

	// allocate memory for samples
	uint16_t *samples = (uint16_t *) malloc(noSamples * noChannels * 2);

	for(uint8_t j = 0; j < noChannels; j++) {
		// select channel to read from
		MUX_select_channel(channels[j]);

		HAL_Delay(5); // wait for output to stabilize

		for(uint16_t i = 0; i < noSamples; i++) {
			// read value from ADC
			HAL_GPIO_WritePin(ADC_CS_GPIOX, ADC_CS, GPIO_PIN_RESET);
			HAL_SPI_Receive(hspi, (uint8_t *) &spiInputBuffer, 1, HAL_MAX_DELAY);
			HAL_GPIO_WritePin(ADC_CS_GPIOX, ADC_CS, GPIO_PIN_SET);

			samples[i * noChannels + j] = (spiInputBuffer & 0x1FFF) >> 1; // ignore first 3 bits and last bit
		}
	}

	return samples;
}

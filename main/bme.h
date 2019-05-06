#ifndef BME_H
#define BME_H

typedef struct
{
	uint32_t pressure;
	int16_t temperature;
	uint16_t humidity;
} bme_data_t;

void bme_init();

bme_data_t bme_read();

#endif /* BME_H */

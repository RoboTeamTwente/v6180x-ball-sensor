#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "vl6180x_def.h"

#include "usart.h"
#include "PuttyInterface.h"

#define myDev   (0x29<<1)
#define ALS_NEWSAMPLE_RANGE_LOWTHRESH 0x21
#define ALS_NEWSAMPLE_RANGE_NEWSAMPLE 0x24

char STATUS_DEBUG;
char SINGLE_SHOT;

uint8_t range;

void WrByte(uint16_t index, uint8_t data);

void RdByte(uint16_t index, uint8_t* data);

void MyDev_SetChipEnable();

void LoadSettings();

void initializeDevice();

void measureRange();

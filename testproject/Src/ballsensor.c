#include "ballsensor.h"

char STATUS_DEBUG = 1;
char SINGLE_SHOT = 0;

void MyDev_SetChipEnable() {
	//STARTUP SEQUENCE:
	//Set GPIO0 to 0
	//Set GPIO0 to 1
	//Wait for a minimum of 400μs

	if(STATUS_DEBUG)
		uprintf("Starting chip enable\n\r");
	HAL_GPIO_WritePin(CHIP_ENABLE_GPIO_Port, CHIP_ENABLE_Pin , (GPIO_PinState)0);
	HAL_Delay(10);
    HAL_GPIO_WritePin(CHIP_ENABLE_GPIO_Port, CHIP_ENABLE_Pin , (GPIO_PinState)1);
    HAL_Delay(1);
    if(STATUS_DEBUG)
    	uprintf("Device booted\n\r");
}


void WrByte(uint16_t index, uint8_t data) {

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, myDev, (uint16_t)index, I2C_MEMADD_SIZE_16BIT, &data, 1, 10000);

	if(status != HAL_OK) {
		uprintf("WrByte failed\r\n");
	}
}

void RdByte(uint16_t index, uint8_t* data) {

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, myDev, (uint16_t)index, I2C_MEMADD_SIZE_16BIT, data, 1, 10000);

	if(status != HAL_OK) {
		uprintf("RdByte failed\r\n");
	}
}

void LoadSettings() {
	WrByte(0x0207, 0x01);
	WrByte(0x0208, 0x01);
	WrByte(0x0096, 0x00);
	WrByte(0x0097, 0xfd);
	WrByte(0x00e3, 0x00);
	WrByte(0x00e4, 0x04);
	WrByte(0x00e5, 0x02);
	WrByte(0x00e6, 0x01);
	WrByte(0x00e7, 0x03);
	WrByte(0x00f5, 0x02);
	WrByte(0x00d9, 0x05);
	WrByte(0x00db, 0xce);
	WrByte(0x00dc, 0x03);
	WrByte(0x00dd, 0xf8);
	WrByte(0x009f, 0x00);
	WrByte(0x00a3, 0x3c);
	WrByte(0x00b7, 0x00);
	WrByte(0x00bb, 0x3c);
	WrByte(0x00b2, 0x09);
	WrByte(0x00ca, 0x09);
	WrByte(0x0198, 0x01);
	WrByte(0x01b0, 0x17);
	WrByte(0x01ad, 0x00);
	WrByte(0x00ff, 0x05);
	WrByte(0x0100, 0x05);
	WrByte(0x0199, 0x05);
	WrByte(0x01a6, 0x1b);
	WrByte(0x01ac, 0x3e);
	WrByte(0x01a7, 0x1f);
	WrByte(0x0030, 0x00);
	// Recommended : Public registers - See data sheet for more detail
	WrByte(SYSTEM_MODE_GPIO1,0x30);
	//WrByte(0x0011, 0x10); // Enables polling for ‘New Sample ready’
	// when measurement completes
	WrByte(0x010a, 0x30); // Set the averaging sample period
	// (compromise between lower noise and
	// increased execution time)
	WrByte(0x003f, 0x46); // Sets the light and dark gain (upper
	// nibble). Dark gain should not be
	// changed.
	WrByte(0x0031, 0xFF); // sets the # of range measurements after
	// which auto calibration of system is
	// performed
	WrByte(0x0040, 0x63); // Set ALS integration time to 100ms
	WrByte(0x002e, 0x01); // perform a single temperature calibration
	// of the ranging sensor
	WrByte(SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09); // Set default ranging inter-measurement
	// period to 10ms
	WrByte(0x003e, 0x31); // Set default ALS inter-measurement period
	// to 500ms
	//WrByte(SYSTEM_MODE_GPIO1,0x08); //set GPIO1 to interrupt output, active high
	//WrByte(SYSTEM_MODE_GPIO1,0x10); //set GPIO1 to interrupt output, active high
	WrByte(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24); // Configures interrupt on ‘New Sample
	// Ready threshold event’

}

void initializeDevice() {
	MyDev_SetChipEnable(); //toggle GPIO0 to reset device

	uint8_t reset, id;

	//CHECK DEVICE ID
	RdByte(IDENTIFICATION_MODEL_ID, &id);
	if(STATUS_DEBUG)
		uprintf("id: %d\r\n", id);
	if(id == 0xB4) {
		if(STATUS_DEBUG)
			uprintf("--> Device recognized!\n\r");
	}
	else {
		uprintf("--> Device not recognized! Exiting...\n\r");
		return;
	}

	//CHECK RESET
	RdByte(SYSTEM_FRESH_OUT_OF_RESET, &reset);
	if(STATUS_DEBUG)
		uprintf("reset: %d\r\n", reset);

	//LOAD A BUNCH OF SETTINGS ONTO DEVICE
	LoadSettings();
	if(STATUS_DEBUG)
		uprintf("settings loaded\r\n");


	//UNSET FRESH OUT OF RESET
	WrByte(SYSTEM_FRESH_OUT_OF_RESET, 0x00);
}

void measureRange()
{
	initializeDevice();

	uint8_t status, range_status;

	// Start a range measurement
	while (!((range_status) & 0x01)){
				RdByte(RESULT_RANGE_STATUS, &range_status);
			}

	if(STATUS_DEBUG)
				uprintf("--> range status: %d\r\n", range_status);

	// Start a range measurement
	//WrByte(SYSRANGE_START, 0x01);
	WrByte(SYSRANGE_START, 0x03);

}



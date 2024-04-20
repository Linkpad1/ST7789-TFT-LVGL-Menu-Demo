/* An STM32 HAL library written for the cst816Ts IC. */

#include "cst816s_hal.h"
#include "main.h"
#ifdef __cplusplus
extern "C"{
#endif

I2C_HandleTypeDef *CS816T_ui2c;
	
/**
 * @brief Initializes the cst816Ts module & Resets the device.
 * @param hi2c User I2C handle pointer.
 */
uint8_t CS816T_Reset(void) {
    HAL_GPIO_WritePin(CS816T_RST_PORT, CS816T_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(CS816T_RST_PORT, CS816T_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(CS816T_RST_PORT, CS816T_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(50);
}

void CS816T_Init(I2C_HandleTypeDef *hi2c) {
	CS816T_Reset();
	CS816T_ui2c = hi2c;
}

void CS816T_SetRegByte(uint8_t regAddr, uint8_t val) {
	uint8_t bytes[2] = { regAddr, val };
	if(HAL_I2C_GetState(CS816T_ui2c) == HAL_I2C_STATE_READY)
	HAL_I2C_Mem_Write(CS816T_ui2c, CS816T_I2C_ADDR<< 1, regAddr , I2C_MEMADD_SIZE_8BIT, &val, sizeof(val),100);
	while (HAL_I2C_GetState(CS816T_ui2c) != HAL_I2C_STATE_READY)
			  {
			  }
}

uint8_t CS816T_GetRegByte(uint8_t regAddr) {
	uint16_t reg = regAddr;
	uint8_t val;
    if(HAL_I2C_GetState(CS816T_ui2c) == HAL_I2C_STATE_READY)
	HAL_I2C_Mem_Read(CS816T_ui2c, CS816T_I2C_ADDR<< 1, reg , I2C_MEMADD_SIZE_8BIT, &val, sizeof(val),100);

    while (HAL_I2C_GetState(CS816T_ui2c) != HAL_I2C_STATE_READY)
			  {
	 	if(HAL_I2C_GetState(CS816T_ui2c) != HAL_I2C_STATE_READY)
	  		     val = 0;
			  }
	return val;
}

uint8_t CS816T_version(void) {
	return (CS816T_GetRegByte(CS816T_FWVer));
}

uint8_t CS816T_Gesture(void) {
	return CS816T_DecodeBCD(CS816T_GetRegByte(CS816T_GestureID));
}

uint8_t CS816T_Touch(void) {
	return CS816T_DecodeBCD(CS816T_GetRegByte(CS816T_ReadTouch));
}

uint8_t CS816T_Xup (void) {
	return CS816T_DecodeBCD_4(CS816T_GetRegByte(CS816T_ReadXup));
}

uint8_t CS816T_Xlo (void) {
	return (CS816T_GetRegByte(CS816T_ReadXlo));
}

uint8_t CS816T_Yup (void) {
	return CS816T_DecodeBCD_4(CS816T_GetRegByte(CS816T_ReadYup));
}

uint8_t CS816T_Ylo (void) {
	return (CS816T_GetRegByte(CS816T_ReadYlo));
}

uint8_t CS816T_GetX(void)
{
	uint16_t X;
	uint8_t Xup;
	uint8_t Xlo;
	Xup = CS816T_Xup();
	Xlo = CS816T_Xlo();
    X = ((Xup<<8)+Xlo);
    return X;
}

uint8_t CS816T_GetY(void)
{
	uint16_t Y;
	uint8_t Yup;
	uint8_t Ylo;
	Yup = CS816T_Yup();
	Ylo = CS816T_Ylo();
    Y = ((Yup<<8)+Ylo);
    return Y;
}
/**
 * @brief Decodes the raw binary value stored in registers to decimal format.
 * @param bin Binary-coded decimal value retrieved from register, 0 to 255.
 * @return Decoded decimal value.
 */
uint8_t CS816T_DecodeBCD(uint8_t bin) {
	return ((bin));
}

uint8_t CS816T_DecodeBCD_4(uint8_t bin) {
	return (((bin & 0xf0) >> 4) * 10) + (bin & 0x0f);
}
/**
 * @brief Encodes a decimal number to binaty-coded decimal for storage in registers.
 * @param dec Decimal number to encode.
 * @return Encoded binary-coded decimal value.
 */
uint8_t CS816T_EncodeBCD(uint8_t dec) {
	return (dec);
}

#ifdef __cplusplus
}
#endif

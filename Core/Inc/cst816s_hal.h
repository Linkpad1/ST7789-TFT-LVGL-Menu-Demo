/* An STM32 HAL library CS816T_GetTouchwritten for the DS1307 real-time clock IC. */
/* Library by @Linkpad
 *
 */
#ifndef CS816T_FOR_STM32_HAL_H
#define CS816T_FOR_STM32_HAL_H
#include "main.h"


#define CS816T_RST_PORT GPIOA
#define CS816T_RST_PIN GPIO_PIN_5

#define CS816T_I2C_ADDR 	0x15

#define CS816T_GestureID   0x01
#define CS816T_ReadTouch   0x02
#define CS816T_ReadXup     0x03
#define CS816T_ReadXlo     0x04
#define CS816T_ReadYup     0x05
#define CS816T_ReadYlo     0x06
#define CS816T_ReadBPCup   0xB0
#define CS816T_ReadBPClo   0xB1

#define CS816T_ChipID      0xA7
#define CS816T_ProjID      0xA8
#define CS816T_FWVer       0xA9
#define CS816T_Sleep       0xFE

#define CS816T_TIMEOUT		1000

/*----------------------------------------------------------------------------*/
extern I2C_HandleTypeDef *CS816T_ui2c;

uint8_t CS816T_Reset(void);
void CS816T_Init(I2C_HandleTypeDef *hi2c);

void CS816T_SetRegByte(uint8_t regAddr, uint8_t val);
uint8_t CS816T_GetRegByte(uint8_t regAddr);
uint8_t CS816T_GetRegByteMulti(uint8_t regAddr, uint8_t length);

uint8_t CS816T_version(void);
uint8_t CS816T_Gesture(void);
uint8_t CS816T_Touch(void);
uint8_t CS816T_Xup (void);
uint8_t CS816T_Xlo (void);
uint8_t CS816T_Xup (void);
uint8_t CS816T_Xlo (void);
uint8_t CS816T_GetX(void);
uint8_t CS816T_GetY(void);

void CS816T_GetTouchInfo(uint8_t GestureID, uint8_t ReadTouch, uint16_t X, uint16_t Y);
uint8_t CS816T_DecodeBCD(uint8_t bin);
uint8_t CS816T_DecodeBCD_4(uint8_t bin);
uint8_t CS816T_EncodeBCD(uint8_t dec);

#endif

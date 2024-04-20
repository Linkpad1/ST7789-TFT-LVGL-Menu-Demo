

#ifndef INC_LCD_LVGL_H_
#define INC_LCD_LVGL_H_

/***********DISP_HOR_RES**********
 *      INCLUDES
 *********************/
#include <stdint.h>
#include "lvgl.h"
#include "st7789.h"


/*********************
 *      DEFINES
 *********************/

#define DISP_VER_RES 280
#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 280
#define SPI_HOST_MAX 1


//lv_display_set_offset(&st7789disp_p, 0, 60);

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void Display_init(int rotation);

void DMA_Handler (void);
#endif /* INC_LCD_LVGL_H_ */

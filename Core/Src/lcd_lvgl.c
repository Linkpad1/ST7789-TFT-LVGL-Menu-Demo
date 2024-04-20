
#include "../../lv_conf.h"
#include "../../lvgl/lvgl.h"

#include "lcd_lvgl.h"

//#include "tft.h"
//#include "user_setting.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "st7789.h"
#include "main.h"
//#include "functions.h"


#define MAX(a,b) ((a)>(b) ? (a):(b))

uint16_t DISP_fb[(MAX(ST7789_HEIGHT,ST7789_WIDTH))]; // LCD FRAME Buffer for 1 ROW



static lv_disp_drv_t * st7789disp_p;
static lv_disp_drv_t disp_drv;
//static int32_t x1_flush;
//static int32_t y1_flush;
//static int32_t x2_flush;
//static int32_t y2_fill;
//static int32_t y_fill_act;
//static const lv_color_t * buf_to_flush;

static lv_color_t disp_buf1[ST7789_WIDTH * 20];
static lv_color_t disp_buf2[ST7789_WIDTH * 20];


static volatile uint32_t t_saved = 0;
void monitor_cb(lv_disp_drv_t * d, uint32_t t, uint32_t p)
{
	t_saved = t;
}



//lv_display_set_buffers(&disp_drv, buf, NULL, 10, LV_DISPLAY_RENDER_MODE_PARTIAL);

static void st7789_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
//These 3 functions are needed by LittlevGL*/



void Display_init(int rotation)
{

	static lv_disp_draw_buf_t buf;
	lv_disp_draw_buf_init(&buf, disp_buf1, disp_buf2, ST7789_WIDTH * 20);
	lv_disp_drv_init(&disp_drv);
	lv_init();
	lv_port_indev_init();

	    disp_drv.draw_buf = &buf;
		disp_drv.flush_cb = st7789_flush_cb;
		disp_drv.monitor_cb = monitor_cb;
		disp_drv.hor_res = ST7789_WIDTH;
		disp_drv.ver_res = ST7789_HEIGHT;
		disp_drv.rotated = rotation;
		lv_disp_drv_register(&disp_drv);


}


static void st7789_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
	 	if(area->x2 < 0) return;
	    if(area->y2 < 0) return;
	    if(area->x1 > LV_HOR_RES_MAX - 1) return;
	    if(area->y1 > LV_VER_RES_MAX - 1) return;

	    //ST7789_Send_Data(area->x1, area->y1, area->x2, area->y2,(uint8_t *)color_p);
	           // uint16_t setColor = (uint16_t)color_p->full;
	    	   // uint8_t data[] = {setColor >> 8, setColor & 0xFF};

	    	   // ST7789_WriteData(data,(area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) * 2);
	    	    //ST7789_DrawImage(area->x1, area->y1, area->x2,  area->y2, data);


	    ST7789_Send_Data_DMA(area->x1, area->y1, area->x2,area->y2, (uint8_t *)color_p);
st7789disp_p=disp_drv;
 color_p++;

}



/*
void st7789_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
	 	if(area->x2 < 0) return;
	    if(area->y2 < 0) return;
	    if(area->x1 > LV_HOR_RES_MAX - 1) return;
	    if(area->y1 > 280 - 1) return;
    int16_t x, y;
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
        	uint16_t setColor = (uint16_t)color_p->full;
        	ST7789_DrawPixel((uint16_t)x,(uint16_t) y,setColor);
            color_p++;
        }
    }
    lv_disp_flush_ready(disp_drv);
}
*/

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Flush a color buffer
 * @param x1 left coordinate of the rectangle
 * @param x2 right coordinate of the rectangle
 * @param y1 top coordinate of the rectangle
 * @param y2 bottom coordinate of the rectangle
 * @param color_p pointer to an array of colors
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
  {
  	 lv_disp_flush_ready(st7789disp_p);
  }

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){}
 // void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi){
//  lv_disp_flush_ready(st7789disp_p);
 // }



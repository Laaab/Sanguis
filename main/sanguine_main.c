#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "byteswap.h"
#include "esp_heap_caps.h"

#include "config.h"
#include "ili9341.h"
#include "stmpe610.h"
#include "camera.h"
#include "ov7670.h"

#define BUFFER_SIZE (320*240*2)

DRAM_ATTR uint8_t buff[BUFFER_SIZE];

static camera_config_t camconf = { 
	.ledc_channel = LEDC_CHANNEL_0,
	.ledc_timer = LEDC_TIMER_0,
	.pin_d0 = CONFIG_D0,
	.pin_d1 = CONFIG_D1,
	.pin_d2 = CONFIG_D2,
	.pin_d3 = CONFIG_D3,
	.pin_d4 = CONFIG_D4,
	.pin_d5 = CONFIG_D5,
	.pin_d6 = CONFIG_D6,
	.pin_d7 = CONFIG_D7,
	.pin_xclk = CONFIG_XCLK,
	.pin_pclk = CONFIG_PCLK,
	.pin_vsync = CONFIG_VSYNC,
	.pin_href = CONFIG_HREF,
	.pin_reset = CONFIG_RESET,
	.xclk_freq_hz = CONFIG_XCLK_FREQ,
	.sample_mode = SM_0A0B_0C0D,
	.pixel_format = CAMERA_PF_RGB565,
	.frame_size = CAMERA_FS_QVGA,
	.i2c_num = CONFIG_I2C_PORT,
};

void peripheral_dir(int dir)
{
	gpio_mode_t mode=dir==DIR_OUTPUT?GPIO_MODE_OUTPUT:GPIO_MODE_INPUT;
	gpio_set_direction(CONFIG_D0, mode);
	gpio_set_direction(CONFIG_D1, mode);
	gpio_set_direction(CONFIG_D2, mode);
	gpio_set_direction(CONFIG_D3, mode);
	gpio_set_direction(CONFIG_D4, mode);
	gpio_set_direction(CONFIG_D5, mode);
	gpio_set_direction(CONFIG_D6, mode);
	gpio_set_direction(CONFIG_D7, mode);
	gpio_set_level(CONFIG_DIR, dir);
}

void peripheral_init()
{
	esp_err_t ret;
	//Initialize 573 control
    gpio_config_t ioconf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
		.pin_bit_mask = 1LL << CONFIG_DIR,
    };
	gpio_config(&ioconf);
	if (rtc_gpio_is_valid_gpio(CONFIG_DIR)) 
        rtc_gpio_deinit(CONFIG_DIR);

	//Initialize the SPI bus
	spi_bus_config_t buscfg={
        .miso_io_num=CONFIG_SPI_MISO,
        .mosi_io_num=CONFIG_SPI_MOSI,
        .sclk_io_num=CONFIG_SPI_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    ret=spi_bus_initialize(CONFIG_SPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);

	//Initialize the I2C bus	
	int i2c_master_port = CONFIG_I2C_PORT;
    i2c_config_t i2sconf;
    i2sconf.mode = I2C_MODE_MASTER;
    i2sconf.sda_io_num = CONFIG_I2C_SDA;
    i2sconf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2sconf.scl_io_num = CONFIG_I2C_SCL;
    i2sconf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2sconf.master.clk_speed = CONFIG_I2C_FREQ;
    i2c_param_config(i2c_master_port, &i2sconf);
    i2c_driver_install(i2c_master_port, i2sconf.mode, 0, 0, 0);
}

void app_main()
{
	int count=0;
    printf("Program start!\n");
	printf("Buffer address:%p\n",buff);
	printf("Buffer end:%p\n",buff+BUFFER_SIZE-1);
	printf("Buffer content:%hhu\n",*(buff+BUFFER_SIZE-1));

	printf("Initialize NVS...\n");
	nvs_flash_init();
	
	printf("Initialize peripheral...\n");
	peripheral_init();
	peripheral_dir(DIR_OUTPUT);

	printf("Initialize touchscreen...\n");
	ili9341_init(CONFIG_SPI_HOST, CONFIG_LCD_CS, CONFIG_LCD_DC, CONFIG_LCD_RST);
	touch_init(CONFIG_SPI_HOST, CONFIG_TOUCH_CS);

	
	printf("Initializing Camera...\n");
	camera_init(&camconf);
	printf("Detecting Camera...\n");
	ovcam_info caminfo;
	ov7670_detect(&caminfo);
	printf("Camera mid: %hu\n", caminfo.mid);
	printf("Camera ver: %hhu\n", caminfo.ver);
	printf("Camera pid: %hhu\n", caminfo.pid);
	printf("Touch id: %hhu\n", touch_detect());
	printf("Initializing OV7670...\n");
	//ov7670_init();
	//camera_skip();
	printf("Start capturing...\n");
	while(1)
	{
		//peripheral_dir(DIR_INPUT);
		//camera_capture((uint32_t *)buff);
		//peripheral_dir(DIR_OUTPUT);
		printf("Captured:%d\n", ++count);
		for(int i=0; i<BUFFER_SIZE; i++)
			buff[i]=~buff[i];
		ili9341_update(0,0,320,240,buff);
		ili9341_update_await();
		printf("Rendered:%d\n", ++count);
		vTaskDelay(1);
	}
}

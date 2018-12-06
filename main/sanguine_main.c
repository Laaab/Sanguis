#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "byteswap.h"

#include "config.h"
#include "ili9341.h"
#include "stmpe610.h"
#include "camera.h"
#include "framebuffer.h"

int pixcount=0;
#define CMASK (0x7BEF)
#define STEP_COUNT 500
static camera_pixelformat_t s_pixel_format;
static camera_config_t config = { 
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
	.pin_sscb_sda = CONFIG_SDA,
	.pin_sscb_scl = CONFIG_SCL,
	.pin_reset = CONFIG_RESET,
	.xclk_freq_hz = CONFIG_XCLK_FREQ,
	.test_pattern_enabled = 0, };

static camera_model_t camera_model;

#define CAMERA_PIXEL_FORMAT CAMERA_PF_RGB565
//#define CAMERA_PIXEL_FORMAT CAMERA_PF_YUV422
#define CAMERA_FRAME_SIZE CAMERA_FS_QVGA

inline uint8_t unpack(int byteNumber, uint32_t value) {
	return (value >> (byteNumber * 8));
}

static void draw_buffer() {
	pixcount=0;
	uint16_t line[2][320];
	int x, y; //, frame=0;
	//Indexes of the line currently being sent to the LCD and the line we're calculating.
	int sending_line = -1;
	int calc_line = 0;

	uint32_t* fbl;    // address of current line/row in framebuffer
	fb_context_t fbc_display;

	int width = camera_get_fb_width();
	int height = camera_get_fb_height();
	uint16_t pixel565 = 0;
	uint16_t pixel565_2 = 0;
	int current_byte_pos = 0;
	int current_pixel_pos = 0;
	int current_line_pos = 0;

	for (y = 0; y < ILI9341_TFTHEIGHT; y++) {
		// begin of line/row
		current_pixel_pos = (y % height) * width;

		fbl = (uint32_t*) framebuffer_pos_32(&fbc_display, current_pixel_pos);

		//Calculate a line, operate on 2 pixels at a time...
		for (x = 0; x < ILI9341_TFTWIDTH; x += 2) {
			// wrap pixels around...
			current_line_pos = x % width;
			current_byte_pos = current_line_pos / 2;

			if (fbl != NULL) {
				// rgb565 direct from OV7670 to ILI9341
				// best to swap bytes here instead of bswap
				uint32_t long2px = 0;
				long2px = fbl[current_byte_pos];
				pixel565 = (unpack(0, long2px) << 8)
						| unpack(1, long2px);
				pixel565_2 = (unpack(2, long2px) << 8)
						| unpack(3, long2px);
				if(80<x&&(line[calc_line][x] & CMASK) == CMASK)pixcount++;
				if(80<x&&(line[calc_line][x+1] & CMASK) == CMASK)pixcount++;

				line[calc_line][x] = __bswap_16(pixel565);

				line[calc_line][x + 1] = __bswap_16(pixel565_2);
			}
		}
		//Finish up the sending process of the previous line, if any
		if (sending_line != -1)
			lcd_send_line_await();
		//Swap sending_line and calc_line
		sending_line = calc_line;
		calc_line = (calc_line == 1) ? 0 : 1;
		//Send the line we currently calculated.
		lcd_send_line(y, line[sending_line]);
	}

	if (sending_line != -1)
		lcd_send_line_await();
}

void peripheral_init()
{
	esp_err_t ret;
	spi_bus_config_t buscfg={
        .miso_io_num=CONFIG_SPI_MISO,
        .mosi_io_num=CONFIG_SPI_MOSI,
        .sclk_io_num=CONFIG_SPI_SCLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
	//Initialize the SPI bus
    ret=spi_bus_initialize(CONFIG_SPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
}

static void display_pretty_colors() 
{
    uint16_t line[2][320];
    int x, y, frame=0;
    //Indexes of the line currently being sent to the LCD and the line we're calculating.
    int sending_line=-1;
    int calc_line=0;
    
    while(1) {
        frame++;
        for (y=0; y<240; y++) {
            //Calculate a line.
            for (x=0; x<320; x++) {
                line[calc_line][x]=((x<<3)^(y<<3)^(frame+x*y));
            }
            //Finish up the sending process of the previous line, if any
            if (sending_line!=-1) lcd_send_line_await();
            //Swap sending_line and calc_line
            sending_line=calc_line;
            calc_line=(calc_line==1)?0:1;
            //Send the line we currently calculated.
            lcd_send_line(y, line[sending_line]);
            //The line is queued up for sending now; the actual sending happens in the
            //background. We can go on to calculate the next line as long as we do not
            //touch line[sending_line]; the SPI sending process is still reading from that.
        }
    }
}

void app_main()
{
	const char TAG[]="CAM";
    printf("Program start!\n");
	printf("Initialize NVS...\n");
	nvs_flash_init();

	gpio_config_t ioconf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
		.pin_bit_mask = 1LL << CONFIG_LCD_CS | 1LL << CONFIG_LCD_DC | 1LL << CONFIG_LCD_RST
		| 1LL << CONFIG_MOTOR_DIR | 1LL << CONFIG_MOTOR_STEP,
    };
	gpio_config(&ioconf);
	if (rtc_gpio_is_valid_gpio(CONFIG_LCD_CS)) 
        rtc_gpio_deinit(CONFIG_LCD_CS);
	if (rtc_gpio_is_valid_gpio(CONFIG_LCD_DC)) 
        rtc_gpio_deinit(CONFIG_LCD_DC);
	if (rtc_gpio_is_valid_gpio(CONFIG_LCD_RST)) 
        rtc_gpio_deinit(CONFIG_LCD_RST);
	if (rtc_gpio_is_valid_gpio(CONFIG_MOTOR_DIR)) 
        rtc_gpio_deinit(CONFIG_LCD_CS);
	if (rtc_gpio_is_valid_gpio(CONFIG_MOTOR_STEP)) 
        rtc_gpio_deinit(CONFIG_LCD_DC);
	if (rtc_gpio_is_valid_gpio(CONFIG_MOTOR_SW)) 
        rtc_gpio_deinit(CONFIG_LCD_RST);

	ioconf.mode = GPIO_MODE_INPUT;
    ioconf.pin_bit_mask = 1LL << CONFIG_MOTOR_SW;
	gpio_config(&ioconf);

	if(gpio_get_level(CONFIG_MOTOR_SW))
	{
		printf("Resetting stepper\n");
		gpio_set_level(CONFIG_MOTOR_DIR, 1);
		while(gpio_get_level(CONFIG_MOTOR_SW))
		{
			gpio_set_level(CONFIG_MOTOR_STEP, 0);
			vTaskDelay(1);
			gpio_set_level(CONFIG_MOTOR_STEP, 1);
			vTaskDelay(1);
		}
		printf("Resetting finished\n");
		while(1);
	}
	else
	{
		printf("Start sucking dye\n");
		gpio_set_level(CONFIG_MOTOR_DIR, 0);
		int i=STEP_COUNT;
		while(--i)
		{
			gpio_set_level(CONFIG_MOTOR_STEP, 0);
			vTaskDelay(1);
			gpio_set_level(CONFIG_MOTOR_STEP, 1);
			vTaskDelay(1);
		}
	}
	
	printf("Initialize LCD...\n");
	peripheral_init();
	lcd_init(CONFIG_SPI_HOST, CONFIG_LCD_CS, CONFIG_LCD_DC, CONFIG_LCD_RST);
	touch_init(CONFIG_SPI_HOST, CONFIG_TOUCH_CS);
	
	printf("Initialize Camera...\n");
	esp_err_t err = camera_probe(&config, &camera_model);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Camera probe failed with error 0x%x", err);
		return;
	}
	if (camera_model == CAMERA_OV7725) {
		ESP_LOGI(TAG, "Detected OV7725 camera, using grayscale bitmap format");
		s_pixel_format = CAMERA_PIXEL_FORMAT;
		config.frame_size = CAMERA_FRAME_SIZE;
	} else if (camera_model == CAMERA_OV7670) {
		ESP_LOGI(TAG, "Detected OV7670 camera");
		s_pixel_format = CAMERA_PIXEL_FORMAT;
		config.frame_size = CAMERA_FRAME_SIZE;
	} else if (camera_model == CAMERA_OV2640) {
		ESP_LOGI(TAG, "Detected OV2640 camera, using JPEG format");
		s_pixel_format = CAMERA_PF_JPEG;
		config.frame_size = CAMERA_FS_VGA;
		config.jpeg_quality = 15;
	} else {
		ESP_LOGE(TAG, "Camera not supported");
		return;
	}
	
	config.pixel_format = s_pixel_format;
	err = camera_init(&config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
		return;
	}
	
	vTaskDelay(1000 / portTICK_RATE_MS);
	
	while(1)
	{
		//printf("Capturing\n");
		camera_run();
		//printf("Rendering\n");
		draw_buffer();
		printf("pixcount:%d\n",pixcount);
		//printf("Detecting\n");
		//if(touch_detect())
			//printf("Touch detected\n");
	}
}

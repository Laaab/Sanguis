#ifndef _CAMERA_H
#define _CAMERA_H
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

typedef enum {
    CAMERA_PF_RGB565 = 0,       //!< RGB, 2 bytes per pixel
    CAMERA_PF_YUV422 = 1,       //!< YUYV, 2 bytes per pixel
    CAMERA_PF_GRAYSCALE = 2,    //!< 1 byte per pixel
    CAMERA_PF_JPEG = 3,         //!< JPEG compressed
    CAMERA_PF_RGB555 = 4,       //!< RGB, 2 bytes per pixel
    CAMERA_PF_RGB444 = 5,       //!< RGB, 2 bytes per pixel
} camera_pixelformat_t;

typedef enum {
    CAMERA_FS_40x30,     	//!< 40x30
    CAMERA_FS_64x32,     	//!< 64x32
    CAMERA_FS_64x64,     	//!< 64x64
    CAMERA_FS_QQCIF,     	//!< 88x72
    CAMERA_FS_QQVGA,    	//!< 160x120
    CAMERA_FS_QQVGA2,   	//!< 128x160
    CAMERA_FS_QCIF,     	//!< 176x144
    CAMERA_FS_HQVGA,    	//!< 220x160
    CAMERA_FS_QVGA,     	//!< 320x240
    CAMERA_FS_CIF,      	//!< 320x240
    CAMERA_FS_VGA,     		//!< 640x480
    CAMERA_FS_SVGA,    		//!< 800x600
    CAMERA_FS_SXGA,     	//!< 1280x1024
    CAMERA_FS_UXGA,     	//!< 1600x1200
} camera_framesize_t;

typedef enum {
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, ...
     */
    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
    SM_0A00_0B00 = 3,
} i2s_sampling_mode_t;

typedef struct {
    gpio_num_t pin_reset;          /*!< GPIO pin for camera reset line */
    gpio_num_t pin_xclk;           /*!< GPIO pin for camera XCLK line */
    gpio_num_t pin_d7;             /*!< GPIO pin for camera D7 line */
    gpio_num_t pin_d6;             /*!< GPIO pin for camera D6 line */
    gpio_num_t pin_d5;             /*!< GPIO pin for camera D5 line */
    gpio_num_t pin_d4;             /*!< GPIO pin for camera D4 line */
    gpio_num_t pin_d3;             /*!< GPIO pin for camera D3 line */
    gpio_num_t pin_d2;             /*!< GPIO pin for camera D2 line */
    gpio_num_t pin_d1;             /*!< GPIO pin for camera D1 line */
    gpio_num_t pin_d0;             /*!< GPIO pin for camera D0 line */
    gpio_num_t pin_vsync;          /*!< GPIO pin for camera VSYNC line */
    gpio_num_t pin_href;           /*!< GPIO pin for camera HREF line */
    gpio_num_t pin_pclk;           /*!< GPIO pin for camera PCLK line */

    int xclk_freq_hz;       /*!< Frequency of XCLK signal, in Hz */
	
	i2s_sampling_mode_t sample_mode;

    ledc_timer_t ledc_timer;        /*!< LEDC timer to be used for generating XCLK  */
    ledc_channel_t ledc_channel;    /*!< LEDC channel to be used for generating XCLK  */

    camera_pixelformat_t pixel_format;
    camera_framesize_t frame_size;
	
	i2c_port_t i2c_num;

} camera_config_t;

esp_err_t sccb_write(const uint8_t addr, const uint8_t reg, const uint8_t data);
esp_err_t sccb_read(const uint8_t addr, const uint8_t reg, uint8_t *data);
esp_err_t camera_init(const camera_config_t* config);
esp_err_t camera_capture(uint32_t *buffer);
void camera_skip();

#endif

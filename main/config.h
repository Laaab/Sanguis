#ifndef CONFIG_H
#define CONFIG_H

#define CONFIG_SPI_HOST VSPI_HOST 
#define CONFIG_SPI_SCLK 18  
#define CONFIG_SPI_MISO 19 
#define CONFIG_SPI_MOSI 23

#define CONFIG_LCD_DC 5
#define CONFIG_LCD_CS 33
#define CONFIG_LCD_RST 14
#define CONFIG_LCD_BKL 15

#define CONFIG_TOUCH_CS 22

#define CONFIG_D0 4
#define CONFIG_D1 5
#define CONFIG_D2 14
#define CONFIG_D3 33
#define CONFIG_D4 22
#define CONFIG_D5 25
#define CONFIG_D6 32
#define CONFIG_D7 2
#define CONFIG_XCLK 21
#define CONFIG_PCLK 36
#define CONFIG_VSYNC 39
#define CONFIG_HREF 34
#define CONFIG_RESET 4
#define CONFIG_XCLK_FREQ 8000000

#define CONFIG_I2C_PORT I2C_NUM_0
#define CONFIG_I2C_SDA 27
#define CONFIG_I2C_SCL 26
#define CONFIG_I2C_FREQ 100000

#define CONFIG_LED 25
#define CONFIG_DIR 0
#define CONFIG_SWITCH 35
#define CONFIG_MOTOR_DIR 32
#define CONFIG_MOTOR_STEP 2

#define DIR_INPUT 0
#define DIR_OUTPUT 1

#endif

#include "stdio.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "stmpe610.h"

spi_device_handle_t touch_spi;
lcd_touch_t touch_pos;
uint8_t touch_buff[4];

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
static void touch_write(const uint8_t cmd, const uint8_t data) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       	//Zero out the transaction
    t.length=16;                 	//Len is in bytes, transaction length is in bits.
    t.flags = SPI_TRANS_USE_TXDATA;
	t.tx_data[0]=cmd;
	t.tx_data[1]=data;
    ret=spi_device_transmit(touch_spi, &t);	//Transmit!
    assert(ret==ESP_OK);            	//Should have had no issues.
}

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
static uint8_t *touch_read(const uint8_t cmd, int len) 
{

	esp_err_t ret;
    spi_transaction_t t;
	memset(&t, 0, sizeof(t));       	//Zero out the transaction
    t.length=8;                 	//Len is in bytes, transaction length is in bits.
    t.flags = SPI_TRANS_USE_TXDATA;
	t.tx_data[0]=cmd;
    ret=spi_device_transmit(touch_spi, &t);	//Transmit!
    assert(ret==ESP_OK);            	//Should have had no issues.
	
    if (len==0||len>4) return NULL;          //no need to send anything
    memset(&t, 0, sizeof(t));       	//Zero out the transaction
    t.length=len*8;                 	//Len is in bytes, transaction length is in bits.
    t.rx_buffer=touch_buff;
    ret=spi_device_transmit(touch_spi, &t);	//Transmit!
    assert(ret==ESP_OK);            	//Should have had no issues.
	return touch_buff;
}

void touch_init(spi_host_device_t host, int cs)
{
	esp_err_t ret;
	
	spi_device_interface_config_t devcfg={
        .clock_speed_hz=SPI_MASTER_FREQ_10M/100,//Clock out at 10 MHz
        .mode=0,                            //SPI mode 0
        .spics_io_num=cs,					//CS pin
        .queue_size=2,                      //We want to be able to queue 7 transactions at a time
    };
	//Attach the touch chip to the SPI bus
    ret=spi_bus_add_device(host, &devcfg, &touch_spi);
    assert(ret==ESP_OK);
	
    touch_write(STMPE610_SYS_CTRL1, 0x02);        // Software chip reset
    vTaskDelay(10 / portTICK_RATE_MS);

    touch_write(STMPE610_SYS_CTRL2, 0x04);        // Temperature sensor clock off, GPIO clock off, touch clock on, ADC clock on

    touch_write(STMPE610_INT_EN, 0x00);           // Don't Interrupt on INT pin

    touch_write(STMPE610_ADC_CTRL1, 0x48);        // ADC conversion time = 80 clock ticks, 12-bit ADC, internal voltage refernce
    vTaskDelay(2 / portTICK_RATE_MS);
    touch_write(STMPE610_ADC_CTRL2, 0x01);        // ADC speed 3.25MHz
    touch_write(STMPE610_GPIO_AF, 0x00);          // GPIO alternate function - OFF
    touch_write(STMPE610_TSC_CFG, 0xE3);          // Averaging 8, touch detect delay 1ms, panel driver settling time 1ms
    touch_write(STMPE610_FIFO_TH, 0x01);          // FIFO threshold = 1
    touch_write(STMPE610_FIFO_STA, 0x01);         // FIFO reset enable
    touch_write(STMPE610_FIFO_STA, 0x00);         // FIFO reset disable
    touch_write(STMPE610_TSC_FRACT_XYZ, 0x07);    // Z axis data format
    touch_write(STMPE610_TSC_I_DRIVE, 0x01);      // max 50mA touchscreen line current
    touch_write(STMPE610_TSC_CTRL, 0x30);         // X&Y&Z, 16 reading window
    touch_write(STMPE610_TSC_CTRL, 0x31);         // X&Y&Z, 16 reading window, TSC enable
    touch_write(STMPE610_INT_STA, 0xFF);          // Clear all interrupts
    touch_write(STMPE610_INT_CTRL, 0x00);         // Level interrupt, disable interrupts
}

uint16_t touch_detect()
{
	return *(uint16_t *)touch_read(0x93,2);
}
 
int touch_fetch()
{
	if (!(*touch_read(STMPE610_TSC_CTRL,1) & 0x80)) return 0;

    // Get touch data
    uint8_t fifo_size = *touch_read(STMPE610_FIFO_SIZE,1);
    while (fifo_size < 2) {
    	if (!(*touch_read(STMPE610_TSC_CTRL,1) & 0x80)) return 0;
        fifo_size = *touch_read(STMPE610_FIFO_SIZE,1);
    }
    while (fifo_size > 120) {
    	if (!(*touch_read(STMPE610_TSC_CTRL,1) & 0x80)) return 0;
        touch_pos.x = *(uint16_t *)touch_read(STMPE610_TSC_DATA_X,2);
        touch_pos.y = *(uint16_t *)touch_read(STMPE610_TSC_DATA_Y,2);
        touch_pos.z = *touch_read(STMPE610_TSC_DATA_Z,1);
        fifo_size = *touch_read(STMPE610_FIFO_SIZE,1);
    }
    for (uint8_t i=0; i < (fifo_size-1); i++) {
        touch_pos.x = *(uint16_t *)touch_read(STMPE610_TSC_DATA_X,2);
        touch_pos.y = *(uint16_t *)touch_read(STMPE610_TSC_DATA_Y,2);
        touch_pos.z = *touch_read(STMPE610_TSC_DATA_Z,1);
    }

    touch_pos.x = 4096 - touch_pos.x;
	return 1;
}

lcd_touch_t *touch_getpos()
{
	return &touch_pos;
}

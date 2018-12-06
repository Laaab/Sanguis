#include "stdio.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "ili9341.h"

spi_device_handle_t lcd_spi;
int lcd_pin_dc;
uint8_t lcd_buff[16];

DRAM_ATTR static const uint8_t init_data[]=
{
	0x39, 0x2C, 0x00, 0x34, 0x02,	// power control A
	0x00, 0XC1, 0X30,				// power control B
	0x64, 0x03, 0X12, 0X81,			// power on sequence control
	0x85, 0x01, 0x79,				// driver timing control A
	0x00, 0x00,						// driver timing control B
	0x20,		// pump ratio control
	0x26,		// power control 1
	0x11,		// power control 2
	0x35, 0x3E,	// vcom control 1
	0xBE,		// vcom control 2
	0x28,		// Memory Access Control
	0x55,		// Pixel Format Set
	0x00, 0x1B, // Frame Rate Control (In Normal Mode/Full Colors)
	0x08,		// Enable 3G
	0x01,		// Gamma Set
	// Positive Gamma Correction
	0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00,
	// Negative Gamma Correction
	0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F,
	0x00, 0x00, 0x00, 0xEF,	// Column Address Set
	0x00, 0x00, 0x01, 0x3f,	// Page Address Set
	0x07,					// Entry Mode Set
	0x0A, 0x82, 0x27, 0x00,	// Display Function Control
};

//Send a command to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_cmd(const uint8_t cmd) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       	//Zero out the transaction
    t.length=8;                     	//Command is 8 bits
    t.tx_buffer=&cmd;               	//The data is the cmd itself
    t.user=(void*)0;					//D/C needs to be set to 0
    ret=spi_device_transmit(lcd_spi, &t);	//Transmit!
    assert(ret==ESP_OK);				//Should have had no issues.
}

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_write(const uint8_t *data, int len) 
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             	//no need to send anything
    memset(&t, 0, sizeof(t));       	//Zero out the transaction
    t.length=len*8;                 	//Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               	//Data
    t.user=(void*)1;                	//D/C needs to be set to 1
    ret=spi_device_transmit(lcd_spi, &t);	//Transmit!
    assert(ret==ESP_OK);            	//Should have had no issues.
}

//Send data to the LCD. Uses spi_device_transmit, which waits until the transfer is complete.
void lcd_read(uint8_t *data, int len) 
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             	//no need to send anything
    memset(&t, 0, sizeof(t));       	//Zero out the transaction
    t.length=len*8;                 	//Len is in bytes, transaction length is in bits.
    t.rx_buffer=data;               	//Data
    t.user=(void*)1;                	//D/C needs to be set to 1
    ret=spi_device_transmit(lcd_spi, &t);	//Transmit!
    assert(ret==ESP_OK);            	//Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will set the D/C line to the value indicated in the user field.
void lcd_pre_transfer_callback(spi_transaction_t *t) 
{
    int dc=(int)t->user;
    gpio_set_level(lcd_pin_dc, dc);
}

void lcd_init(spi_host_device_t host,int cs, int dc, int rst)
{
	esp_err_t ret;
	uint8_t lcd_buff[16];
	uint8_t *init_data_ptr=init_data;
	
	lcd_pin_dc = dc;
	gpio_set_direction(dc, GPIO_MODE_OUTPUT);
    gpio_set_direction(rst, GPIO_MODE_OUTPUT);
	spi_device_interface_config_t devcfg={
        .clock_speed_hz=SPI_MASTER_FREQ_20M,//Clock out at 10 MHz
        .mode=0,                            //SPI mode 0
        .spics_io_num=cs,					//CS pin
        .queue_size=7,                      //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
	//Attach the LCD to the SPI bus
    ret=spi_bus_add_device(host, &devcfg, &lcd_spi);
    assert(ret==ESP_OK);
	
	//Reset the display
    gpio_set_level(rst, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(rst, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
	
	//Read device id
	lcd_cmd(ILI9341_RDDID);
	lcd_read(lcd_buff, 3);
	printf("find LCD ID: %08X\n", *(uint32_t *)lcd_buff);
	
	// start initialization sequence
	lcd_cmd(ILI9341_PWCTRA);
	lcd_write(init_data_ptr, 5);
	init_data_ptr+=5;
	
	lcd_cmd(ILI9341_PWCTRB);
	lcd_write(init_data_ptr, 3);
	init_data_ptr+=3;
	
	lcd_cmd(ILI9341_PWRONSEQ);
	lcd_write(init_data_ptr, 4);
	init_data_ptr+=4;
	
	// driver control
	lcd_cmd(ILI9341_DTCTRA);
	lcd_write(init_data_ptr, 3);
	init_data_ptr+=3;
	
	lcd_cmd(ILI9341_DTCTRB);
	lcd_write(init_data_ptr, 2);
	init_data_ptr+=2;
	
	lcd_cmd(ILI9341_PRCTR);
	lcd_write(init_data_ptr, 1);
	init_data_ptr++;
	
	// power control
	lcd_cmd(ILI9341_PWCTR1);
	lcd_write(init_data_ptr, 1);
	init_data_ptr++;
	
	lcd_cmd(ILI9341_PWCTR2);
	lcd_write(init_data_ptr, 1);
	init_data_ptr++;
	
	lcd_cmd(ILI9341_VMCTR1);
	lcd_write(init_data_ptr, 2);
	init_data_ptr+=2;
	
	lcd_cmd(ILI9341_VMCTR2);
	lcd_write(init_data_ptr, 1);
	init_data_ptr++;
	
	// memory access control
	lcd_cmd(ILI9341_MADCTL);
	lcd_write(init_data_ptr, 1);
	init_data_ptr++;
	
	// set format
	lcd_cmd(ILI9341_PIXFMT);
	lcd_write(init_data_ptr, 1);
	init_data_ptr++;	
	
	lcd_cmd(ILI9341_FRMCTR1);
	lcd_write(init_data_ptr, 2);
	init_data_ptr+=2;
	
	lcd_cmd(ILI9341_EN3G);
	lcd_write(init_data_ptr, 1);
	init_data_ptr++;
	
	// gamma correct
	lcd_cmd(ILI9341_GAMMASET);
	lcd_write(init_data_ptr, 1);
	init_data_ptr++;
	
	lcd_cmd(ILI9341_GMCTRP1);
	lcd_write(init_data_ptr, 15);
	init_data_ptr+=15;
	
	lcd_cmd(ILI9341_GMCTRN1);
	lcd_write(init_data_ptr, 15);
	init_data_ptr+=15;
	
	// memory format
	lcd_cmd(ILI9341_CASET);
	lcd_write(init_data_ptr, 4);
	init_data_ptr+=4;
	
	lcd_cmd(ILI9341_PASET);
	lcd_write(init_data_ptr, 4);
	init_data_ptr+=4;
	
	lcd_cmd(ILI9341_RAMWR);
	
	lcd_cmd(ILI9341_ETMOD);
	lcd_write(init_data_ptr, 1);
	init_data_ptr++;
	
	lcd_cmd(ILI9341_DFUNCTR);
	lcd_write(init_data_ptr, 4);
	
	lcd_cmd(ILI9341_SLPOUT);
	vTaskDelay(100 / portTICK_RATE_MS);
	lcd_cmd(ILI9341_DISPON);
	vTaskDelay(100 / portTICK_RATE_MS);
}

//Fast image transfer

//To send a line we have to send a command, 2 data bytes, another command, 2 more data bytes and another command before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction because the D/C line needs to be toggled in the middle.) This routine queues these commands up so they get sent as quickly as possible.
void lcd_send_line(int ypos, uint16_t *line)
{
    esp_err_t ret;
    int x;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[6];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized variables. We allocate them on the stack, so we need to re-init them each call.
    for (x=0; x<6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0]=0x2A;           //Column Address Set
    trans[1].tx_data[0]=0;              //Start Col High
    trans[1].tx_data[1]=0;              //Start Col Low
    trans[1].tx_data[2]=(320)>>8;       //End Col High
    trans[1].tx_data[3]=(320)&0xff;     //End Col Low
    trans[2].tx_data[0]=0x2B;           //Page address set
    trans[3].tx_data[0]=ypos>>8;        //Start page high
    trans[3].tx_data[1]=ypos&0xff;      //start page low
    trans[3].tx_data[2]=(ypos+1)>>8;    //end page high
    trans[3].tx_data[3]=(ypos+1)&0xff;  //end page low
    trans[4].tx_data[0]=0x2C;           //memory write
    trans[5].tx_buffer=line;            //finally send the line data
    trans[5].length=320*2*8;            //Data length, in bits
    trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag

    //Queue all transactions.
    for (x=0; x<6; x++) {
        ret=spi_device_queue_trans(lcd_spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    // When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to finish because we may as well spend the time calculating the next line. When that is done, we can call send_line_finish, which will wait for the transfers to be done and check their status.
}


void lcd_send_line_await() 
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 6 transactions to be done and get back the results.
    for (int x=0; x<6; x++) {
        ret=spi_device_get_trans_result(lcd_spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}

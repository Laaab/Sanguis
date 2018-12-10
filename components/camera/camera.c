#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/lldesc.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/periph_ctrl.h"
#include "driver/i2c.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "camera.h"

#define ACK_CHECK_EN	0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS	0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL			0x0              /*!< I2C ack value */
#define NACK_VAL		0x1              /*!< I2C nack value */

#define TAG				"Cam"

typedef void (*dma_filter_t)(const uint32_t* src, lldesc_t* dma_desc, uint32_t* dst);

const int resolution[][2] = {
        { 40, 30 }, /* 40x30 */
        { 64, 32 }, /* 64x32 */
        { 64, 64 }, /* 64x64 */
        { 88, 72 }, /* QQCIF */
        { 160, 120 }, /* QQVGA */
        { 128, 160 }, /* QQVGA2*/
        { 176, 144 }, /* QCIF  */
        { 240, 160 }, /* HQVGA */
        { 320, 240 }, /* QVGA  */
        { 352, 288 }, /* CIF   */
        { 640, 480 }, /* VGA   */
        { 800, 600 }, /* SVGA  */
        { 1280, 1024 }, /* SXGA  */
        { 1600, 1200 }, /* UXGA  */
};

const uint32_t I2S_CONF_RESET_FLAGS = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
const uint32_t I2S_LC_CONF_RESET_FLAGS = I2S_IN_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;

gpio_num_t pin_vsync;
uint32_t *cam_buffer;
size_t width;
size_t height;
size_t buf_size;
size_t bytes_per_pixel;

lldesc_t *dma_desc;
uint32_t **dma_buf;
size_t dma_desc_count;
uint32_t dma_desc_mask;
size_t dma_received_count;
size_t dma_filtered_count;
size_t dma_sample_count;

typedef union {
	uint32_t l;
	uint16_t m[2];
	uint8_t s[4];
}int_union;

dma_filter_t dma_filter;
intr_handle_t i2s_intr_handle;
intr_handle_t vsync_intr_handle;
QueueHandle_t data_ready;
SemaphoreHandle_t frame_ready;
TaskHandle_t dma_filter_task_handle;
camera_pixelformat_t pixel_format;

i2c_port_t i2c_num;

static void IRAM_ATTR dma_filter_task(void *pvParameters);
static void IRAM_ATTR dma_filter_compact(const uint32_t* src, lldesc_t* dma_desc, uint32_t* dst);
static void IRAM_ATTR dma_filter_sparse(const uint32_t* src, lldesc_t* dma_desc, uint32_t* dst);
static void IRAM_ATTR i2s_isr(void* arg);
static void IRAM_ATTR gpio_isr(void* arg);

esp_err_t sccb_write(const uint8_t addr, const uint8_t reg, const uint8_t data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t sccb_read(const uint8_t addr, const uint8_t reg, uint8_t *data)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr | I2C_MASTER_WRITE, I2C_MASTER_ACK);
	i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if(ret!=ESP_OK) return ret;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, addr | I2C_MASTER_READ, I2C_MASTER_ACK);
	i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
    return ret;
}

static inline void i2s_conf_reset()
{
	// Toggle some reset bits in LC_CONF register
    I2S0.lc_conf.val |= I2S_LC_CONF_RESET_FLAGS;
    I2S0.lc_conf.val &= ~I2S_LC_CONF_RESET_FLAGS;
    // Toggle some reset bits in CONF register
    I2S0.conf.val |= I2S_CONF_RESET_FLAGS;
    I2S0.conf.val &= ~I2S_CONF_RESET_FLAGS;
    while (I2S0.state.rx_fifo_reset_back);
}

esp_err_t camera_init(const camera_config_t* config)
{
	width=resolution[config->frame_size][0];
	height=resolution[config->frame_size][0];
	pin_vsync=config->pin_vsync;
	pixel_format=config->pixel_format;
	i2c_num=config->i2c_num;
	
    //!> Configure input GPIOs
    gpio_num_t pins[] = {
            config->pin_d7,
            config->pin_d6,
            config->pin_d5,
            config->pin_d4,
            config->pin_d3,
            config->pin_d2,
            config->pin_d1,
            config->pin_d0,
            config->pin_vsync,
            config->pin_href,
            config->pin_pclk,
			config->pin_xclk,
			config->pin_reset,
    };
	
    gpio_config_t conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
			.pin_bit_mask =0 
    };
	
    for (int i = 0; i < sizeof(pins) / sizeof(gpio_num_t); ++i) {
		if (rtc_gpio_is_valid_gpio(pins[i])) 
            rtc_gpio_deinit(pins[i]);
        conf.pin_bit_mask |= 1LL << pins[i];
    }
	gpio_config(&conf);
	
	gpio_set_direction(config->pin_xclk, GPIO_MODE_OUTPUT);
	gpio_set_direction(config->pin_reset, GPIO_MODE_OUTPUT);

	//!> start xclk signal with led control module
	periph_module_enable(PERIPH_LEDC_MODULE);

    ledc_timer_config_t timer_conf;
    timer_conf.duty_resolution = LEDC_TIMER_1_BIT;
    timer_conf.freq_hz = config->xclk_freq_hz;
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num = config->ledc_timer;
    esp_err_t err = ledc_timer_config(&timer_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
        return err;
    }

    ledc_channel_config_t ch_conf;
    ch_conf.channel = config->ledc_channel;
    ch_conf.timer_sel = config->ledc_timer;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.duty = 1;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.gpio_num = config->pin_xclk;
	ch_conf.hpoint = 0;
    err = ledc_channel_config(&ch_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
        return err;
    }
	
	//!> Set up I2S peripheral in camera mode
    // Route input GPIOs to I2S peripheral using GPIO matrix
    gpio_matrix_in(config->pin_d0, I2S0I_DATA_IN0_IDX, false);
    gpio_matrix_in(config->pin_d1, I2S0I_DATA_IN1_IDX, false);
    gpio_matrix_in(config->pin_d2, I2S0I_DATA_IN2_IDX, false);
    gpio_matrix_in(config->pin_d3, I2S0I_DATA_IN3_IDX, false);
    gpio_matrix_in(config->pin_d4, I2S0I_DATA_IN4_IDX, false);
    gpio_matrix_in(config->pin_d5, I2S0I_DATA_IN5_IDX, false);
    gpio_matrix_in(config->pin_d6, I2S0I_DATA_IN6_IDX, false);
    gpio_matrix_in(config->pin_d7, I2S0I_DATA_IN7_IDX, false);
    gpio_matrix_in(config->pin_vsync, I2S0I_V_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(config->pin_href, I2S0I_H_ENABLE_IDX, false);
    gpio_matrix_in(config->pin_pclk, I2S0I_WS_IN_IDX, false);

    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);
	i2s_conf_reset();
	
    // Enable slave mode (sampling clock is external)
    I2S0.conf.rx_slave_mod = 1;
    // Enable parallel mode
    I2S0.conf2.lcd_en = 1;
    // Use HSYNC/VSYNC/HREF to control sampling
    I2S0.conf2.camera_en = 1;
    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 1;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = 2;
    // FIFO will sink data to DMA
    I2S0.fifo_conf.dscr_en = 1;
    // FIFO configuration
    I2S0.fifo_conf.rx_fifo_mod = config->sample_mode;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;
    I2S0.conf_chan.rx_chan_mod = 1;
    // Clear flags which are used in I2S serial mode
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;
    I2S0.timing.val = 0;

    // Allocate I2S interrupt, keep it disabled
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE,
    ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
        &i2s_isr, NULL, &i2s_intr_handle);
	
	//!> Allocate DMA buffers
	assert(width % 4 == 0);
	if(config->sample_mode==SM_0A0B_0C0D){
		bytes_per_pixel=4;
		dma_filter=dma_filter_compact;
	}
	else{
		bytes_per_pixel=8;
		dma_filter=dma_filter_sparse;
	}
    size_t line_size = width * bytes_per_pixel;
    ESP_LOGD(TAG, "Line width (for DMA): %d bytes", line_size);
    size_t dma_per_line = 1;
	dma_desc_mask=1;
    buf_size = line_size;
    while (buf_size >= 4096) {
        buf_size /= 2;
        dma_per_line *= 2;
		dma_desc_mask<<=1;
    }
    dma_desc_count = dma_per_line * 4;
	dma_desc_mask = (dma_desc_mask << 2) - 1;
    ESP_LOGD(TAG, "DMA buffer size: %d, DMA buffers per line: %d", buf_size, dma_per_line);
    ESP_LOGD(TAG, "DMA buffer count: %d", dma_desc_count);

    dma_buf = (uint32_t**) malloc(sizeof(uint32_t*) * dma_desc_count);
    if (dma_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    dma_desc = (lldesc_t*) malloc(sizeof(lldesc_t) * dma_desc_count);
    if (dma_desc == NULL) {
        return ESP_ERR_NO_MEM;
    }
	
	//!> configure DMA ring
	dma_sample_count = 0;
    for (int i = 0; i < dma_desc_count; i++) {
        ESP_LOGD(TAG, "Allocating DMA buffer #%d, size=%d", i, buf_size);
        uint32_t *buf = (uint32_t*) malloc(buf_size);
        if (buf == NULL) {
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGV(TAG, "dma_buf[%d]=%p", i, buf);
		dma_buf[i]=buf;

        lldesc_t* pd = &dma_desc[i];
        pd->length = buf_size;
        if (config->sample_mode == SM_0A0B_0B0C &&
            (i + 1) % dma_per_line == 0) {
            pd->length -= 4;
        }
		dma_sample_count += pd->length / 4;
        pd->size = pd->length;
        pd->owner = 1;
        pd->sosf = 1;
        pd->buf = (uint8_t*) buf;
        pd->offset = 0;
        pd->empty = 0;
        pd->eof = 1;
        pd->qe.stqe_next = &dma_desc[(i + 1) & dma_desc_mask];
    }
	
	//!> create capturing semaphores and task
	data_ready = xQueueCreate(16, sizeof(size_t));
    frame_ready = xSemaphoreCreateBinary();
    if (data_ready == NULL || frame_ready == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
	if (!xTaskCreatePinnedToCore(&dma_filter_task, "dma_filter", 8128, NULL, 20, &dma_filter_task_handle, 1)) {
       ESP_LOGE(TAG, "Failed to create DMA filter task");
       err = ESP_ERR_NO_MEM;
       goto fail;
    }
    ESP_LOGD(TAG, "Initializing GPIO interrupts");
    gpio_set_intr_type(pin_vsync, GPIO_INTR_NEGEDGE);
    gpio_intr_enable(pin_vsync);
    if (vsync_intr_handle == NULL) {
      ESP_LOGD(TAG, "Initializing GPIO ISR Register");
      err = gpio_isr_register(&gpio_isr, (void*) TAG,
              ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_IRAM,
              &vsync_intr_handle);
      if (err != ESP_OK) {
          ESP_LOGE(TAG, "gpio_isr_register failed (%x)", err);
          goto fail;
      }
    } else {
      ESP_LOGD(TAG, "Skipping GPIO ISR Register, already enabled...");
    }

	//!> resetting camera
    gpio_set_level(config->pin_reset, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(config->pin_reset, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    return ESP_OK;
	
	
fail:
    if (data_ready) {
        vQueueDelete(data_ready);
    }
    if (frame_ready) {
        vSemaphoreDelete(frame_ready);
    }
    if (dma_filter_task_handle) {
        vTaskDelete(dma_filter_task_handle);
    }
    if (dma_buf) {
        for (int i = 0; i < dma_desc_count; ++i) {
            free(dma_buf[i]);
        }
    }
    free(dma_buf);
    free(dma_desc);
    ESP_LOGE(TAG, "Init Failed");
    return err;
}

esp_err_t camera_capture(uint32_t *buffer)
{
	cam_buffer=buffer;
	assert(cam_buffer);
	// wait for vsync
    ESP_LOGD(TAG, "Waiting for end of frame");
    while (gpio_get_level(pin_vsync) == 0);
    while (gpio_get_level(pin_vsync) != 0);
    ESP_LOGD(TAG, "Got VSYNC");

    dma_received_count = 0;
    dma_filtered_count = 0;
    esp_intr_disable(i2s_intr_handle);
    i2s_conf_reset();

    I2S0.rx_eof_num = dma_sample_count;
    I2S0.in_link.addr = (uint32_t) &dma_desc[0];
    I2S0.in_link.start = 1;
    I2S0.int_clr.val = I2S0.int_raw.val;
    I2S0.int_ena.val = 0;
    I2S0.int_ena.in_done = 1;
    esp_intr_enable(i2s_intr_handle);

    if (pixel_format == CAMERA_PF_JPEG) {
        esp_intr_enable(vsync_intr_handle);
    }

    I2S0.conf.rx_start = 1;
	
	xSemaphoreTake(frame_ready, portMAX_DELAY);
	return ESP_OK;
}

void camera_skip()
{
    while (gpio_get_level(pin_vsync) != 0);
	while (gpio_get_level(pin_vsync) == 0);
    while (gpio_get_level(pin_vsync) != 0);
}

static void i2s_stop()
{
    esp_intr_disable(i2s_intr_handle);
    esp_intr_disable(vsync_intr_handle);
	i2s_conf_reset();
    I2S0.conf.rx_start = 0;
    size_t val = SIZE_MAX;
    BaseType_t higher_priority_task_woken;
    xQueueSendFromISR(data_ready, &val, &higher_priority_task_woken);
}

static void IRAM_ATTR dma_filter_task(void *pvParameters)
{
    while (true) {
        size_t buf_idx;
        xQueueReceive(data_ready, &buf_idx, portMAX_DELAY);
	    if (buf_idx == SIZE_MAX) {
	        xSemaphoreGive(frame_ready);
	        continue;
	    }
		uint32_t* pfb = cam_buffer+dma_filtered_count*buf_size/8;
	    ESP_LOGV(TAG, "dma_flt: pos=%p ", pfb);
	    (*dma_filter)(dma_buf[buf_idx], &dma_desc[buf_idx], pfb);
	    dma_filtered_count++;
	    ESP_LOGV(TAG, "dma_flt: count=%d ", dma_filtered_count);
    }
}

static void IRAM_ATTR dma_filter_compact(const uint32_t* src, lldesc_t* dma_desc, uint32_t* dst)
{
	for (int i = 0; i < buf_size/8; ++i) {
		((uint16_t *)dst)[0] = ((uint16_t *)src)[0]>>8 | ((uint16_t *)src)[1];
		((uint16_t *)dst)[1] = ((uint16_t *)src)[2]>>8 | ((uint16_t *)src)[3];
		src += 2;
		dst += 1;
	}
}

static void IRAM_ATTR dma_filter_sparse(const uint32_t* src, lldesc_t* dma_desc, uint32_t* dst)
{
     // manually unrolling 4 iterations of the loop here
    for (size_t i = 0; i < buf_size/8; ++i) {
		((uint16_t *)dst)[0] = ((uint16_t *)src)[0]>>8 | ((uint16_t *)src)[2];
		((uint16_t *)dst)[0] = ((uint16_t *)src)[4]>>8 | ((uint16_t *)src)[6];
		src += 4;
		dst += 1;
    }
     // the final sample of a line in SM_0A0B_0B0C sampling mode needs special handling
    if ((dma_desc->length & 0x7) != 0) {
		((uint16_t *)dst)[0] = ((uint16_t *)src)[0]>>8 | ((uint16_t *)src)[2];
		((uint16_t *)dst)[0] = ((uint16_t *)src)[4]>>8 | ((uint16_t *)src)[5];
    }
}

static void IRAM_ATTR signal_dma_buf_received(bool* need_yield)
{
    size_t dma_desc_cur = dma_received_count & dma_desc_mask;
    dma_received_count++;
    BaseType_t higher_priority_task_woken;
    BaseType_t ret = xQueueSendFromISR(data_ready, &dma_desc_cur, &higher_priority_task_woken);
    if (ret != pdTRUE) {
        ESP_EARLY_LOGW(TAG, "queue send failed (%d), dma_received_count=%d", ret, dma_received_count);
    }
    *need_yield = (ret == pdTRUE && higher_priority_task_woken == pdTRUE);
}

// detect end of bitmap image
static void IRAM_ATTR i2s_isr(void* arg)
{
    I2S0.int_clr.val = I2S0.int_raw.val;
    bool need_yield;
    signal_dma_buf_received(&need_yield);
    ESP_EARLY_LOGV(TAG, "isr, cnt=%d", dma_received_count);
    if (dma_received_count == height * dma_desc_count / 4) {
        i2s_stop();
    }
    if (need_yield) {
        portYIELD_FROM_ISR();
    }
}

// detect end of jpg image
static void IRAM_ATTR gpio_isr(void* arg)
{
    GPIO.status1_w1tc.val = GPIO.status1.val;
    GPIO.status_w1tc = GPIO.status;
    bool need_yield = false;
    ESP_EARLY_LOGV(TAG, "gpio isr, cnt=%d", dma_received_count);
    if (gpio_get_level(pin_vsync) == 0 && dma_received_count > 0) {
        signal_dma_buf_received(&need_yield);
        i2s_stop();
    }
	if (need_yield) {
		portYIELD_FROM_ISR();
	}
}

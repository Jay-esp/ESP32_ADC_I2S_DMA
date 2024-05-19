// this code demonstrates how to sample two ADC1 ports at high speed over the internal I2S bus using DMA
// since the ADC inputs are multiplexed they can never be read at the same time
// by setting ESP32 registers it is possible to read them sequentially
// the output is one byte first channel, one byte second channel and so on
// output is formatted for the serial plotter option in the arduino IDE
// feel free to use the code for your projects
// adapt to the channels you need on ADC1
// Jef Collin 2024


#include <driver/i2s.h>
#include "soc/syscon_reg.h"
#include "driver/adc.h"

// I2S
#define I2S_SAMPLE_RATE (176400) // Max sampling frequency = 277.777 kHz
#define ADC_INPUT (ADC1_CHANNEL_0)
#define ADC_INPUT2 (ADC1_CHANNEL_3)
#define I2S_DMA_BUF_LEN (1024)

size_t bytes_read;

uint16_t chan0;
uint16_t chan1;

// setup the I2S ADC DMA
void i2sInit() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate =  I2S_SAMPLE_RATE,              // The format of the signal using ADC_BUILT_IN
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  adc1_config_channel_atten(ADC_INPUT, ADC_ATTEN_DB_11);

  adc1_config_channel_atten(ADC_INPUT2, ADC_ATTEN_DB_11);

  adc1_config_width(ADC_WIDTH_12Bit);

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);

  i2s_set_adc_mode(ADC_UNIT_1, ADC_INPUT);

  i2s_adc_enable(I2S_NUM_0);

  // Scan multiple channels.
  SET_PERI_REG_BITS(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR1_PATT_LEN, 1, SYSCON_SARADC_SAR1_PATT_LEN_S);

  // This 32 bit register has 4 bytes for the first set of channels to scan.
  // Each byte consists of:
  // [7:4] Channel
  // [3:2] Bit Width; 3=12bit, 2=11bit, 1=10bit, 0=9bit
  // [1:0] Attenuation; 3=11dB, 2=6dB, 1=2.5dB, 0=0dB
  WRITE_PERI_REG(SYSCON_SARADC_SAR1_PATT_TAB1_REG, 0x0F3F0000);

  // The raw ADC data is written to DMA in inverted form. Invert back.
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);

}

void setup() {
  Serial.begin(115200);

  // Initialize the I2S peripheral
  i2sInit();
}

void loop() {
  // create and clear the buffer
  uint16_t bufferl[I2S_DMA_BUF_LEN] = {0};

  while (1) {
    // fill buffer from DMA
    i2s_read(I2S_NUM_0, &bufferl, sizeof(bufferl), &bytes_read, portMAX_DELAY);
    // split data in 2 channels
    // The 4 high bits are the channel, and the data is inverted
    for (int i = 0; i < bytes_read / 2; i = i + 2) {
      if ((bufferl[i] & 0xF000) >> 12 == 0) {
        chan0 = (bufferl[i] & 0x0FFF);
      }
      else {
        chan1 = (bufferl[i] & 0x0FFF);
      }
      if ((bufferl[i + 1] & 0xF000) >> 12 == 0) {
        chan0 = (bufferl[i + 1] & 0x0FFF);
      }
      else {
        chan1 = (bufferl[i + 1] & 0x0FFF);
      }
      Serial.printf("CH0");
      Serial.printf(":%d ", chan0);
      Serial.print(",");
      Serial.printf("CH1");
      Serial.printf(":%d ", chan1);
      Serial.printf("\n");
    }
  }
}

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>  // Include the UDP library
#include "soc/i2s_reg.h"

i2s_chan_handle_t rx_handle;
i2s_chan_config_t chan_cfg;

WiFiUDP udp;  // Create an instance of the UDP class

const char *ssid = "MINE5213";
const char *password = "qwertyuiop";
uint32_t des_buffer[1024];
size_t bytes_readed;

// UDP server address
// const char *serverIP = "192.168.137.1";  // Replace with your server IP
// const char *serverIP = "188.245.33.234";
const char *serverIP = "172.27.8.172";
// const int serverPort = 12345;            // Choose an appropriate port for your server
const int serverPort = 1111;        // Choose an appropriate port for your server
const uint32_t sampleRate = 16000;  // 44.1 kHz sample rate

int num_msgs = 0;

void setup() {
  Serial.begin(115200);

  chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = 6,
    .dma_frame_num = 240,
    .auto_clear = false,
  };

  // Initialize I2S
  i2s_new_channel(&chan_cfg, NULL, &rx_handle);
  i2s_std_config_t std_rx_cfg = {
    .clk_cfg = {
      .sample_rate_hz = sampleRate,
      .clk_src = I2S_CLK_SRC_PLL_160M,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    },
    .slot_cfg = {
      .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
      .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
      .slot_mode = I2S_SLOT_MODE_STEREO,
      .slot_mask = I2S_STD_SLOT_BOTH,
      .ws_width = 32,
      .ws_pol = false,
      .bit_shift = false,
      .msb_right = false,
    },
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = GPIO_NUM_32,
      .ws = GPIO_NUM_25,
      .dout = I2S_GPIO_UNUSED,
      .din = GPIO_NUM_33,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  i2s_channel_init_std_mode(rx_handle, &std_rx_cfg);
  i2s_channel_enable(rx_handle);
  REG_SET_BIT(I2S_TIMING_REG(I2S_NUM_0), BIT(9));
  REG_SET_BIT(I2S_CONF_REG(I2S_NUM_0), I2S_RX_MSB_SHIFT);

  // Initialize WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Begin the UDP communication
  udp.begin(12345);  // Choose the local port (same port for sending and receiving)

  while (!Serial.available()) {
    // Capture audio from I2S
    i2s_channel_read(rx_handle, des_buffer, 1024, &bytes_readed, portMAX_DELAY);
    // Serial.print("I2S readed ... ");
    // Send the captured audio over UDP if data is available
    if (bytes_readed > 0) {
      udp.beginPacket(serverIP, serverPort);
      udp.write((const uint8_t *)des_buffer, bytes_readed);  // Send the audio data as a packet
      udp.endPacket();
    }
    // Optional: Print the number of packets sent for debugging
    num_msgs++;
  }
  Serial.print("Packets sent: ");
  Serial.println(num_msgs);
}





void loop() {
}

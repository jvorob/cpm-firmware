#include <ESP32SPISlave.h>
// Library docs: https://github.com/hideakitai/ESP32SPISlave/tree/main
// Devboard pinout https://www.instructables.com/ESP32-Internal-Details-and-Pinout/
// Wroom Module datasheet https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf


ESP32SPISlave slave;

static constexpr size_t BUFFER_SIZE = 64;
static constexpr size_t QUEUE_SIZE = 1;
uint8_t tx_buf[BUFFER_SIZE] {1, 2, 3, 4, 5, 6, 7, 8};
uint8_t rx_buf[BUFFER_SIZE] {0, 0, 0, 0, 0, 0, 0, 0};



void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);

    delay(2000);

    slave.setDataMode(SPI_MODE0);   // default: SPI_MODE0
    slave.setQueueSize(QUEUE_SIZE); // default: 1    
    slave.begin(HSPI);  // default: HSPI (please refer README for pin assignments)

    Serial.println("start spi slave");

}

void loop() {    
    // initialize tx/rx buffers
    //initializeBuffers(tx_buf, rx_buf, BUFFER_SIZE);

    // start and wait to complete one BIG transaction (same data will be received from slave)
    const size_t rx_bytes = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);

    Serial.printf("Got %d bytes\n", rx_bytes);
//    Serial.printf("Binary: ");
//    for(int i = 0; i < rx_bytes; i++) {
//      Serial.printf("%02x ", rx_buf[i]);
//    }
//    Serial.printf("\n");

    rx_buf[rx_bytes]=0;
    Serial.printf("String: '%s'\n", rx_buf);

}

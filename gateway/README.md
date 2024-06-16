This is the firmware for the gateway that recieves LoRa packets from
my artemis-based sensor board and sends it up to the internet.

(This is a deeply cursed abomination that was hacked together to get a working
demo, and should not be replicated under any circumstances). Both of these use 
arduino 1.8, and use board libraries scavenged from the internet.


The LoRa receiver is a Heltec Cubecell AB02A board [link](https://heltec.org/project/htcc-ab02a/).
It listens for LoRa packets, then forwards them verbatim over SPI.
It also sends a heartbeat packet at power-on.
Firwmare in `htcc-ab02a/test_htcc_lora_rx`

The ESP32 chip (a nodeMCU 32S?) connects to WiFi,
listens for transmissions over SPI bus, then uploads them into a [grist](www.getgrist.com)
document. Main firmware is in `esp32_gateway/esp32_spi_wifi_grist_bridge/`
NOTE: this one depends one a "SECRETS.h" file containing #defines for WIFI passwords
and API Keys, which is excluded by .gitignore.


Data format expected is like "DAT1|AM01|0.000|0.000|0.000"

TODO: Gateway photo
TODO: pinouts? protocol details?

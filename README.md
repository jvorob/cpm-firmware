
# Cathodic Protection Monitoring - Firmware

_Janet Vorobyeva_

This repo contains the firmware for my Cathodic Protection Monitoring project, as part of [CSE237D](https://kastner.ucsd.edu/ryan/cse-237d-embedded-system-design/), Embedded System Design.

The goal of the project was to develop an energy-harvesting sensor node to monitor the health a 
 [cathodic protection](https://en.wikipedia.org/wiki/Cathodic_protection) system. This sensor
can sustain itself on a trickle of power (<250 uW), and so can run directly
from the electric potential it's designed to measure. It transmits measurements several times a day
over a [LoRa](https://en.wikipedia.org/wiki/LoRa) radio, (range of up to 5-15km, depending on conditions)


![Photo of the sensor board](board_photo.jpg)

# Firmware:

This repo contains firmware for both the sensor node and a LoRa gateway to bridge it to the internet.

### Sensor node (custom board)
- Sparkfun Artemis module (based on Ambiq Apollo3 microcontroller)
- ADP5091 Energy harvesting chip
- RFM95W LoRa module, for low power radio transmission
- Designed to be extremely low power, able to survive on a power budget ~100uW,
  and transmit several radio packets a day with voltage measurements.

Firmware is in `artemis_cpm_firmware/`

Hardware is in a separate repo: [cpm-hardware](https://github.com/jvorob/cpm-hardware)


### LoRa Gateway 
Currently this is a 
Heltec AB02A LoRa node duct-taped to a 
nodeMCU ESP32-S (?) module (devboard around a ESP-WROOM-32 module?)
to act as a gateway, uploading received LoRa packets to a [grist](getgrist.com) document.

(this part was meant as a quick and dirty solution, and it very much is)


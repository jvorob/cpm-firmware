Janet Vorobyeva
2024.06.15

# Overview

This repo contains the firmware for my Cathodic Protection Monitoring project.

Designed to monitor a [cathodic protection](https://en.wikipedia.org/wiki/Cathodic_protection) system,
being able to be inserted into a CP Test station and both monitor and power itself off of the electric
potential at the station.

There are several parts to this

![Photo of the sensor board](board_photo.jpg)

### Sensor node (custom board)
- Sparkfun Artemis module (based on Ambiq Apollo3 microcontroller)
- ADP5091 Energy harvesting chip
- RFM95W LoRa module, for low power radio transmission
- Designed to be extremely low power, able to survive on a power budget ~100uW,
  and transmit several radio packets a day with voltage measurements.

Firmware in `artemis_cpm_firmware/`
Hardware in a separate repo: [cpm-hardware](https://github.com/jvorob/cpm-hardware)


### LoRa Gateway 
Currently this is a 
Heltec AB02A LoRa node duct-taped to a 
nodeMCU ESP32-S (?) module (devboard around a ESP-WROOM-32 module?)
to act as a gateway, uploading received LoRa packets to a [grist](getgrist.com) document.

(this part was meant as a quick and dirty solution, and it very much is)


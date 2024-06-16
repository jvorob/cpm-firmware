// For heltec htcc-AB02A

// Janet Vorobyeva - 2024.05.23
// Test LoRA recieveing
// Heltec library sources in ~/.arduino15/packages/CubeCell/hardware/CubecCell/1.5.0
// some stuff in 
//     - libraries/LoRa/src/LoRaWan_APP.h
//     - libraries/LoraWan102/src/
//       - radio, LoraWan_102.h

// NOTE: THE SPI output is very janky
// Currently outputs the following pin assignments:
// CS:   11
// SCK:  10
// MISO: 9
// MOSI: 8
// I think these aren't the internal pin names, but they match the HTCC-AB02A pinout diagram
// https://resource.heltec.cn/download/CubeCell/HTCC-AB02A/HTCC-AB02A_PinoutDiagram.pdf

#define CHIP_ID "HT01"


#include "Arduino.h"
#include <SPI.h>

// JV: LED state stuff
long ledOffAfter = 0; // if >0, turn off LED once millis() exceeds this
//long lastMillis = 0;
//bool ledState = 0;



// ========= LORA STUFF??


#include "LoRaWan_APP.h"
/*
 * set LoraWan_RGB to 1,the RGB active in loraWan
 * RGB red means sending;
 * RGB green means received done;
 */
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

//#define RF_FREQUENCY                                915000000 // Hz
#define RF_FREQUENCY                                902300000 // JV: 902.3 MHz, US915 channel 0


#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        12        // Same for Tx and Rx 
                                                              // (JV: copied to match default from RFM95W)
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 1024 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

int16_t txNumber;

int16_t rssi,rxSize;

bool lora_idle = true;
// ============ END LORA STUFF


#define MY_SCK
#define MY_MISO
#define MY_MOSI

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW); 


  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW); 


  //lastMillis = millis();
  
  Serial.printf("Hello world\n");




  //pinMode(GPIO4, OUTPUT);
  //SPI1.begin(10,9,8,4); 
  //update frequency to 10000000;
  //SPI1.setFrequency(1000000);
  
  
  Serial.printf("SCK %d, MISO %d, MOSI %d, CS %d\n",SCK1,MISO1,MOSI1,GPIO1);
  
  SPI1.begin(SCK1,MISO1,MOSI1,GPIO4);// sck, miso, mosi, nss
  SPI1.setFrequency(1000000); //1MHz. Up to 10MHz should be fine?
  
  //uint8_t data[] = {0x42, 0x37};
  uint8_t data[] = "HB1|" CHIP_ID "\0";
  //SPI1.transfer(0x42);
  //SPI1.transfer(0x37);
  

  delay(100);
  Serial.printf("Sending SPI Hearbeat: '%s'\n", data);
  digitalWrite(GPIO4, LOW); 
  SPI1.transfer(data, sizeof(data));
  //Wait until all trnasmitted?
  while(SPI_2_SpiUartGetTxBufferSize() != 0);
  //SPI transfer doesn't seem to be blocking? Let's just delay for it
  delay(1);
  digitalWrite(GPIO4, HIGH); 
  delay(10);



  
  // LORA INIT
  Serial.printf("Initializing LORA\n", data);
  txNumber=0;
  rssi=0;    
  RadioEvents.RxDone = OnRxDone;
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                 0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

}

void loop() {
  long currTime = millis();

  //if (currTime - lastMillis > 1000) {
  //  ledState = !ledState;
  //  digitalWrite(LED, ledState);
  //  lastMillis += 1000;
  //}
  if(ledOffAfter > 0 && millis() > ledOffAfter) {
    digitalWrite(LED, LOW);
    ledOffAfter = 0;
  }


  if(lora_idle)
  {
    turnOffRGB(); //in LoRaWan_APP
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0);
  }

}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    rssi=rssi;
    rxSize=size;
    memcpy(rxpacket, payload, size );
    rxpacket[size]='\0';
    turnOnRGB(COLOR_RECEIVED,0);
    Radio.Sleep( );
    Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n",rxpacket,rssi,rxSize);
    lora_idle = true;

    // JV: Blink LED
    digitalWrite(LED, HIGH);
    ledOffAfter = millis() + 25; 


    

    Serial.printf("Sending %d bytes over SPI tx\n", rxSize);
    

    digitalWrite(GPIO4, LOW); 
    //SPI1.transfer(0x42);
    //SPI1.transfer(0x37);
    
    Serial.printf("Sending SPI Packet: '%s'\n", rxpacket);
    SPI1.transfer((uint8_t*)rxpacket, rxSize);
    //Wait until all trnasmitted?
    while(SPI_2_SpiUartGetTxBufferSize() != 0);
    //SPI transfer doesn't seem to be blocking? Let's just delay for it
    delay(1);
    digitalWrite(GPIO4, HIGH); 

    
}

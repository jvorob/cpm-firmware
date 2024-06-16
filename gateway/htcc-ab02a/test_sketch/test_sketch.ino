// For heltec htcc-AB02A

// Janet Vorobyeva - 2024.05.23
// Test LoRA recieveing
// Heltec library sources in ~/.arduino15/packages/CubeCell/hardware/CubecCell/1.5.0
// some stuff in 
//     - libraries/LoRa/src/LoRaWan_APP.h
//     - libraries/LoraWan102/src/
//       - radio, LoraWan_102.h
#include "Arduino.h"

long lastMillis = 0;
bool ledState = 0;



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

#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

int16_t txNumber;

int16_t rssi,rxSize;

bool lora_idle = true;
// ============ END LORA STUFF

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW); 


  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW); 


  lastMillis = millis();
  
  Serial.printf("Hello world\n");



  // LORA INIT
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

  if (currTime - lastMillis > 1000) {
    ledState = !ledState;
    digitalWrite(LED, ledState);
    lastMillis += 1000;
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
}

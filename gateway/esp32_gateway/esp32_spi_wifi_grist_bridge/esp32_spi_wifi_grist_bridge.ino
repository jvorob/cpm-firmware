/**
 * BasicHTTPClient.ino
 *
 *  Created on: 24.05.2015
 *
 */



//NOTE: Program this as "ESP32 Dev Module"
// Depends on ESP32 SPI Slave library

#include "SECRETS.h" // Excluded via .gitignore, has API keys and such

#include <Arduino.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>

#define USE_SERIAL Serial

WiFiMulti wifiMulti;


// ====== SPI STUFF
#include <ESP32SPISlave.h>

ESP32SPISlave slave;

static constexpr size_t BUFFER_SIZE = 64;
static constexpr size_t QUEUE_SIZE = 1;
uint8_t spi_tx_buf[BUFFER_SIZE] {1, 2, 3, 4, 5, 6, 7, 8};
uint8_t spi_rx_buf[BUFFER_SIZE] {0, 0, 0, 0, 0, 0, 0, 0};
char spi_raw_buf[BUFFER_SIZE] {};


// CONFIGS
#define LED 2


/*
const char* ca = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIEkjCCA3qgAwIBAgIQCgFBQgAAAVOFc2oLheynCDANBgkqhkiG9w0BAQsFADA/\n" \
"MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n" \
"DkRTVCBSb290IENBIFgzMB4XDTE2MDMxNzE2NDA0NloXDTIxMDMxNzE2NDA0Nlow\n" \
"SjELMAkGA1UEBhMCVVMxFjAUBgNVBAoTDUxldCdzIEVuY3J5cHQxIzAhBgNVBAMT\n" \
"GkxldCdzIEVuY3J5cHQgQXV0aG9yaXR5IFgzMIIBIjANBgkqhkiG9w0BAQEFAAOC\n" \
"AQ8AMIIBCgKCAQEAnNMM8FrlLke3cl03g7NoYzDq1zUmGSXhvb418XCSL7e4S0EF\n" \
"q6meNQhY7LEqxGiHC6PjdeTm86dicbp5gWAf15Gan/PQeGdxyGkOlZHP/uaZ6WA8\n" \
"SMx+yk13EiSdRxta67nsHjcAHJyse6cF6s5K671B5TaYucv9bTyWaN8jKkKQDIZ0\n" \
"Z8h/pZq4UmEUEz9l6YKHy9v6Dlb2honzhT+Xhq+w3Brvaw2VFn3EK6BlspkENnWA\n" \
"a6xK8xuQSXgvopZPKiAlKQTGdMDQMc2PMTiVFrqoM7hD8bEfwzB/onkxEz0tNvjj\n" \
"/PIzark5McWvxI0NHWQWM6r6hCm21AvA2H3DkwIDAQABo4IBfTCCAXkwEgYDVR0T\n" \
"AQH/BAgwBgEB/wIBADAOBgNVHQ8BAf8EBAMCAYYwfwYIKwYBBQUHAQEEczBxMDIG\n" \
"CCsGAQUFBzABhiZodHRwOi8vaXNyZy50cnVzdGlkLm9jc3AuaWRlbnRydXN0LmNv\n" \
"bTA7BggrBgEFBQcwAoYvaHR0cDovL2FwcHMuaWRlbnRydXN0LmNvbS9yb290cy9k\n" \
"c3Ryb290Y2F4My5wN2MwHwYDVR0jBBgwFoAUxKexpHsscfrb4UuQdf/EFWCFiRAw\n" \
"VAYDVR0gBE0wSzAIBgZngQwBAgEwPwYLKwYBBAGC3xMBAQEwMDAuBggrBgEFBQcC\n" \
"ARYiaHR0cDovL2Nwcy5yb290LXgxLmxldHNlbmNyeXB0Lm9yZzA8BgNVHR8ENTAz\n" \
"MDGgL6AthitodHRwOi8vY3JsLmlkZW50cnVzdC5jb20vRFNUUk9PVENBWDNDUkwu\n" \
"Y3JsMB0GA1UdDgQWBBSoSmpjBH3duubRObemRWXv86jsoTANBgkqhkiG9w0BAQsF\n" \
"AAOCAQEA3TPXEfNjWDjdGBX7CVW+dla5cEilaUcne8IkCJLxWh9KEik3JHRRHGJo\n" \
"uM2VcGfl96S8TihRzZvoroed6ti6WqEBmtzw3Wodatg+VyOeph4EYpr/1wXKtx8/\n" \
"wApIvJSwtmVi4MFU5aMqrSDE6ea73Mj2tcMyo5jMd6jmeWUHK8so/joWUoHOUgwu\n" \
"X4Po1QYz+3dszkDqMp4fklxBwXRsW10KXzPMTZ+sOPAveyxindmjkW8lGy+QsRlG\n" \
"PfZ+G6Z6h7mjem0Y+iWlkYcV4PIWL1iwBi8saCbGS5jN2p8M+X+Q7UNKEkROb3N6\n" \
"KOqkqm57TH2H3eDJAkSnh6/DNFu0Qg==\n" \
"-----END CERTIFICATE-----\n";
*/

int i = 0;

#include <esp_wifi.h>
void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void setup() {

  USE_SERIAL.begin(115200);

  USE_SERIAL.println();
  USE_SERIAL.println();
  USE_SERIAL.println();


  slave.setDataMode(SPI_MODE0);   // default: SPI_MODE0
  slave.setQueueSize(QUEUE_SIZE); // default: 1    
  slave.begin(HSPI);  // default: HSPI (please refer README for pin assignments)

  Serial.println("start spi slave");

  pinMode(LED, OUTPUT);

  for (uint8_t t = 4; t > 0; t--) {
    USE_SERIAL.printf("[SETUP] WAIT %d...\n", t);
    USE_SERIAL.flush();
    delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();
  readMacAddress();


  wifiMulti.addAP(SECRET_WIFI_UNAME, SECRET_WIFI_PW);

}

void my_strncpy(char *from, char *to, int n) {
  int i;
  for(i = 0; i < n-1; i++) {
    to[i] = from[i];
    if(to[i] == 0){break;}
  }
  to[i] = 0; //if we terminated on a 0, or we reached n-1, nullterminate
}

void loop() {
  i++; 


    Serial.printf("Waiting for SPI txn\n");
    const size_t rx_bytes = slave.transfer(spi_tx_buf, spi_rx_buf, BUFFER_SIZE);
    spi_rx_buf[rx_bytes] = 0;

    // Turn on LED while processing a packet
    digitalWrite(LED, HIGH);
    delay(50);
    Serial.printf("Got %d bytes over SPI\n", rx_bytes);
    Serial.printf("String: '%s'\n", spi_rx_buf);

    char * p = (char*)spi_rx_buf;
    char * end = (char*)&(spi_rx_buf[rx_bytes]);

    // We're going to modify spi_rx_buf, so copy a fresh version here
    my_strncpy((char*)spi_rx_buf, spi_raw_buf, rx_bytes+1); // null terms the last char, so add 1

    // turns |s into 0s for parsing
    for(;p<end;p++) {
      if(*p == '|') { *p = '\0'; }
    }
    
    const int MAX_FIELDS=16;
    String fields[MAX_FIELDS] = {};
    int num_fields = 0;
    // Turn them all into strings
    p = (char*)spi_rx_buf;
    while(p < end) {
      fields[num_fields] = String(p);
      Serial.printf("Found field: '%s'\n", fields[num_fields].c_str());
      num_fields++;
      if(num_fields == MAX_FIELDS) {break;}
      
      while(p<end && *p != 0) { // Keep going until we find a 0
        p++;
      }
      // We've found a 0, advance to the next field
      p++;
    }




  
  // wait for WiFi connection
  if ((wifiMulti.run() == WL_CONNECTED)) {   

    HTTPClient http;

    USE_SERIAL.print("[HTTP] begin...\n");
    // configure traged server and url
    //http.begin("https://www.howsmyssl.com/a/check", ca); //HTTPS


    http.begin("https://docs.getgrist.com/api/docs/" SECRET_GRIST_DOCID "/tables/CPS_DATA/records?limit=2");

    http.addHeader("Authorization","Bearer " SECRET_GRIST_APIKEY);
    http.addHeader("Content-Type","application/json");


    USE_SERIAL.print("[HTTP] GET...\n");
    // start connection and send HTTP header
    int httpCode = http.GET();

    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      USE_SERIAL.printf("[HTTP] GET... code: %d\n", httpCode);

      // file found at server
      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        USE_SERIAL.println(payload);
      }
    } else {
      USE_SERIAL.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();


    USE_SERIAL.print("\n ====== POSTING RECORD =========\n");
    http.begin("https://docs.getgrist.com/api/docs/" SECRET_GRIST_DOCID "/tables/CPS_DATA/records");

    http.addHeader("Authorization","Bearer " SECRET_GRIST_APIKEY);
    http.addHeader("Content-Type","application/json");

//    char tx_buf[1024]={};
//    int len = snprintf(tx_buf,1024, "{\"records\":[{\"fields\":{"
//    "\"src_id\": \"esp32_gateway_01\","
//    "\"data_type\": \"heartbeat\","
//    "\"vin\": \"%d\""
//    //"\"\": \"\","
//    "}}]}", i);


   char *type="", *id="", *vin="", *vbatt="", *vdd="";
   // Payload should be like 
   if(num_fields < 2) {
    type="ERR";
    id="N/A";
   } else {
      type = (char*)fields[0].c_str();
      id =   (char*)fields[1].c_str();
      if(fields[0] == "DAT1" && num_fields >= 5) {
        vin =   (char*)fields[2].c_str();
        vbatt = (char*)fields[3].c_str();
        vdd =   (char*)fields[4].c_str();
      }
   }
   

   char tx_buf[1024]={};
    int len = snprintf(tx_buf,1024, "{\"records\":[{\"fields\":{"
    "\"gateway_id\": \"esp32_gateway_01\","
    "\"type_code\": \"%s\","
    "\"dev_id\": \"%s\","
    "\"v_in\": \"%s\","
    "\"v_batt\": \"%s\","
    "\"v_vdd\": \"%s\","
    "\"raw\": \"%s\""
    //"\"\": \"\","
    "}}]}", type, id,
    vin, vbatt, vdd,
    spi_raw_buf);



    
    httpCode = http.POST((uint8_t*)tx_buf, len);

    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      USE_SERIAL.printf("[HTTP] POST... code: %d\n", httpCode);

        String payload = http.getString();
        USE_SERIAL.println(payload);
    } else {
      USE_SERIAL.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
  } else {
    USE_SERIAL.printf("Error: not connected to WIFI\n");

    // Signal no WIFI error
    for(int i = 0; i < 3; i++) {
      digitalWrite(LED, LOW);
      delay(200);
      digitalWrite(LED, HIGH);
      delay(200);
    }
  }

  
  // Done processing Packet
  digitalWrite(LED, LOW);

  delay(1000);
}

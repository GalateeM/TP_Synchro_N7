// DecaDuinoReceiver
// This sketch shows how to use the DecaDuino library to receive messages over the UWB radio.
// The sketch prints the received bytes in HEX; it can be used as a frame sniffer.
// by Adrien van den Bossche <vandenbo@univ-tlse2.fr>
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details

#include <SPI.h>
#include <DecaDuino.h>

#define MAX_FRAME_LEN 120
uint8_t rxData[MAX_FRAME_LEN];
uint8_t txT3[MAX_FRAME_LEN];
uint16_t rxLen;
int rxFrames;
uint64_t t1, t2, t3, t4;
int nb_rx_messages;
#ifdef ARDUINO_DWM1001_DEV
DecaDuino decaduino(SS1, DW_IRQ);
#define LED_ON LOW
#define LED_OFF HIGH
#else
DecaDuino decaduino;
#define LED_ON HIGH
#define LED_OFF LOW
#endif


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // Internal LED (pin LED_BUILTIN on DecaWiNo board)
  Serial.begin(115200); // Init Serial port
  #ifndef ARDUINO_DWM1001_DEV
  SPI.setSCK(14); // Set SPI clock pin (pin 14 on DecaWiNo board)
  #endif

  // Init DecaDuino and blink if initialisation fails
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
    while(1) { digitalWrite(LED_BUILTIN, LED_ON); delay(50); digitalWrite(LED_BUILTIN, LED_OFF); delay(50); }
  }
  // Set RX buffer and enable RX
  decaduino.setRxBuffer(rxData, &rxLen);
  decaduino.plmeRxEnableRequest();
  rxFrames = 0;
  nb_rx_messages = 0;
  t1 = 0;
  t2 = 0;
  t3 = 0;
  t4 = 0;
}


void loop()
{

  decaduino.engine();
  
  if ( decaduino.rxFrameAvailable() ) {
    if(rxData[0] == 30) {
      if(nb_rx_messages == 0 && rxData[1] == uint8_t(0)) {
        // Receive a first message from master
        Serial.print("ICI");
        Serial.print("#"); Serial.print(++rxFrames); Serial.print(" ");
        Serial.print(rxLen);
        Serial.println("bytes received: |");
        t2 = decaduino.getLastRxTimestamp();
        Serial.print("T2 : ");
        decaduino.printUint64(t2);
        Serial.println("");
        nb_rx_messages = 1;
      }

      if(nb_rx_messages == 1 && rxData[1] == uint8_t(1)) {
        // Receive a second message from master
        Serial.print("#"); Serial.print(++rxFrames); Serial.print(" ");
        Serial.print(rxLen);
        Serial.println("bytes received: |");
        t1 = decaduino.decodeUint64(rxData + 2);
        
        Serial.print("T1 : ");
        decaduino.printUint64(t1);
        Serial.println("");
        nb_rx_messages = 2;

        // envoi du message
        decaduino.plmeRxDisableRequest();
        delay(500);
        txT3[0] = uint8_t(30); //panID
        decaduino.pdDataRequest(txT3, 1);
        while ( !decaduino.hasTxSucceeded() ) {decaduino.engine();};
        decaduino.plmeRxEnableRequest();
        t3 = decaduino.getLastTxTimestamp();
        Serial.print("T3 : ");
        decaduino.printUint64(t3);
        Serial.println("");
      }

      if(nb_rx_messages == 2 && rxData[1] == uint8_t(2)) {
        // Receive a third message from master
        Serial.print("#"); Serial.print(++rxFrames); Serial.print(" ");
        Serial.print(rxLen);
        Serial.println("bytes received: |");
        t4 = decaduino.decodeUint64(rxData + 2);
      
        Serial.print("T4 : ");
        decaduino.printUint64(t4);
        Serial.println();

        //calcul
        if(t2 < t1) {
          uint64_t MS_diff = t1 - t2;
          uint64_t SM_diff = t3 - t4;
          uint64_t offset = (MS_diff - SM_diff) / 2;
          Serial.print("offset négatif : ");
          decaduino.printUint64(offset);
          Serial.println();
        } else {
          uint64_t MS_diff = t2 - t1;
          uint64_t SM_diff = t4 - t3;
          uint64_t offset = (MS_diff - SM_diff) / 2;
          Serial.print("offset positif : ");
          decaduino.printUint64(offset);
          Serial.println();
        }       
        //reset nb_messages
        nb_rx_messages = 0;
      }
      
    }
    decaduino.plmeRxEnableRequest(); // Always renable RX after a frame reception  
  }

  
}



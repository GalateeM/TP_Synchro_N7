#include <SPI.h>
#include <DecaDuino.h>

#define MAX_FRAME_LEN 120
uint8_t rxData[MAX_FRAME_LEN];
uint8_t txAck[2];
uint16_t rxLen;
int rxFrames;
#ifdef ARDUINO_DWM1001_DEV
DecaDuino decaduino(SS1, DW_IRQ);
#define LED_ON LOW
#define LED_OFF HIGH
#else
DecaDuino decaduino;
#define LED_ON HIGH
#define LED_OFF LOW
#endif
int ack_lost = 0; //simule le fait de perdre des acks


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
}


void loop()
{

  decaduino.engine();
  
  // WAIT FOR RECEPTION
  if ( decaduino.rxFrameAvailable() ) {
    Serial.print(rxData[0]);
    if(rxData[0] == 30) {
      digitalWrite(LED_BUILTIN, LED_ON);
      Serial.print("#"); Serial.print(++rxFrames); Serial.print(" ");
      Serial.print(rxLen);
      Serial.print("bytes received: |");
      for (int i=0; i<rxLen; i++) {
        if ( rxData[i]<16 ) Serial.print("0");
        Serial.print(rxData[i], HEX);
        Serial.print("|");
      }
      Serial.println();

      decaduino.plmeRxDisableRequest();

      if(ack_lost % 5 == 0) {
        // SEND ACK
        digitalWrite(LED_BUILTIN, LED_ON);
        //personalisation des champs : octet 0 = panID, octet 1 = frame ID
        txAck[0] = 30;
        txAck[1] = rxData[1];

        decaduino.pdDataRequest(txAck, 2);
          
        while ( !decaduino.hasTxSucceeded() ) {decaduino.engine();};
        digitalWrite(LED_BUILTIN, LED_OFF);
        Serial.print("ACK : ");
        Serial.println(rxData[1]);
      }

      ++ack_lost;
      
      
    }
    
    decaduino.plmeRxEnableRequest(); // Always renable RX after a frame reception
    digitalWrite(LED_BUILTIN, LED_OFF);
  }
}



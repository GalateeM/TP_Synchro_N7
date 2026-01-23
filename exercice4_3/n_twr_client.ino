// DecaDuinoTWR_client
// A simple implementation of the TWR protocol, client side
// Contributors: Adrien van den Bossche, Réjane Dalcé, Ibrahim Fofana, Robert Try, Thierry Val
// This sketch is a part of the DecaDuino Project - please refer to the DecaDuino LICENCE file for licensing details
// This sketch implements the skew correction published in "Nezo Ibrahim Fofana, Adrien van den Bossche, Réjane
// Dalcé, Thierry Val, "An Original Correction Method for Indoor Ultra Wide Band Ranging-based Localisation System"
// https://arxiv.org/pdf/1603.06736.pdf

#include <SPI.h>
#include <DecaDuino.h>

// Timeout parameters
#define TIMEOUT_WAIT_START_SENT 20 //ms
#define TIMEOUT_WAIT_ACK 100 //ms

// Ranging period parameter
#define RANGING_PERIOD 500 //ms

// TWR client states state machine enumeration: see state diagram on documentation for more details
enum { TWR_ENGINE_STATE_INIT, TWR_ENGINE_STATE_WAIT_START_SENT, TWR_ENGINE_STATE_MEMORISE_T1,
TWR_ENGINE_STATE_WAIT_ACK, TWR_ENGINE_STATE_MEMORISE_T4, TWR_ENGINE_STATE_WAIT_DATA_REPLY, 
TWR_ENGINE_STATE_EXTRACT_T2_T3 };

// Message types of the TWR protocol
#define TWR_MSG_TYPE_UNKNOWN 0
#define TWR_MSG_TYPE_START 1
#define TWR_MSG_TYPE_DATA_REPLY 3

#define NB_SERVER 2

uint64_t t1, t2[NB_SERVER], t3[NB_SERVER], t4[NB_SERVER];
uint64_t mask = 0xFFFFFFFFFF;
int32_t tof;

#ifdef ARDUINO_DWM1001_DEV
DecaDuino decaduino(SS1, DW_IRQ);
#else
DecaDuino decaduino;
#endif
uint8_t txData[128];
uint8_t rxData[128];
uint16_t rxLen;
int state;
uint32_t timeout;

int received_data_reply;
uint64_t rtt, response_time;



void setup()
{
  pinMode(13, OUTPUT); // Internal LED (pin 13 on DecaWiNo board)
  Serial.begin(115200); // Init Serial port
#ifndef ARDUINO_DWM1001_DEV
  SPI.setSCK(14); // Set SPI clock pin (pin 14 on DecaWiNo board)
#endif
  // Init DecaDuino and blink if initialisation fails
  if ( !decaduino.init() ) {
    Serial.println("decaduino init failed");
    while(1) { digitalWrite(13, HIGH); delay(50); digitalWrite(13, LOW); delay(50); }
  }

  // Set RX buffer
  decaduino.setRxBuffer(rxData, &rxLen);
  state = TWR_ENGINE_STATE_INIT;

  // Print top table colomns
  Serial.println("ToF\td\tToF_sk\td_sk");
}

void loop()
{
  decaduino.engine();

  float distance;
  
  switch (state) {
   
    case TWR_ENGINE_STATE_INIT:
      delay(RANGING_PERIOD); // Wait to avoid medium flooding between two rangings or if a ranging fails
      decaduino.plmeRxDisableRequest();
      Serial.println("New TWR");
      txData[0] = 30;
      txData[1] = TWR_MSG_TYPE_START;
      decaduino.pdDataRequest(txData, 2);
      timeout = millis() + TIMEOUT_WAIT_START_SENT;
      state = TWR_ENGINE_STATE_WAIT_START_SENT;
      received_data_reply = 0;
      break;

    case TWR_ENGINE_STATE_WAIT_START_SENT:
      if ( millis() > timeout ) {
        state = TWR_ENGINE_STATE_INIT;
      } else {
        if ( decaduino.hasTxSucceeded() ) {
          state = TWR_ENGINE_STATE_MEMORISE_T1;
      Serial.println("Succeed");
        }
      }
      break;

    case TWR_ENGINE_STATE_MEMORISE_T1:
      t1 = decaduino.getLastTxTimestamp();
      timeout = millis() + TIMEOUT_WAIT_ACK;
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_WAIT_DATA_REPLY;
      break;


    case TWR_ENGINE_STATE_WAIT_DATA_REPLY:
      if ( millis() > timeout ) {
        state = TWR_ENGINE_STATE_INIT;
        Serial.println("timeout");
      } else {
        if ( decaduino.rxFrameAvailable() ) {
          if ( rxData[0] == 30 && rxData[1] == TWR_MSG_TYPE_DATA_REPLY ) {
            state = TWR_ENGINE_STATE_MEMORISE_T4;
          } else {
            decaduino.plmeRxEnableRequest();
          	state = TWR_ENGINE_STATE_WAIT_DATA_REPLY;
          }
        }
      }
      break;

    case TWR_ENGINE_STATE_MEMORISE_T4:
      t4[received_data_reply] = decaduino.getLastRxTimestamp();
      decaduino.plmeRxEnableRequest();
      state = TWR_ENGINE_STATE_EXTRACT_T2_T3;
      break;

    case TWR_ENGINE_STATE_EXTRACT_T2_T3:
    {
      uint64_t &current_t2 = t2[received_data_reply];
      uint64_t &current_t3 = t3[received_data_reply];
      uint64_t &current_t4 = t4[received_data_reply];
      Serial.print(received_data_reply);
      Serial.print("->     ");
      t2[received_data_reply] = decaduino.decodeUint40(&rxData[2]);
      t3[received_data_reply] = decaduino.decodeUint40(&rxData[7]);
      rtt = (current_t4 - t1) & mask;
      response_time =  (current_t3 - current_t2) & mask;
      
      tof = (rtt - response_time)/2;
      distance = tof*RANGING_UNIT;
      Serial.print(tof);
      Serial.print("\t");
      Serial.print(distance);
      tof = (  rtt  - ( (1+1.0E-6*decaduino.getLastRxSkew()) *(response_time) ))/2;
     
      
      distance = tof*RANGING_UNIT;
      Serial.print("\t");
      Serial.print(tof);
      Serial.print("\t");
      Serial.println(distance);
      
      ++received_data_reply;
      timeout = millis() + TIMEOUT_WAIT_ACK;
      state = ((received_data_reply < NB_SERVER) ? TWR_ENGINE_STATE_WAIT_DATA_REPLY : TWR_ENGINE_STATE_INIT);
      break;
    }
      

    default:
      state = TWR_ENGINE_STATE_INIT;
      break;
  }
}



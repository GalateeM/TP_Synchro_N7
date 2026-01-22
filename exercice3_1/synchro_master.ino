#include <SPI.h>
#include <DecaDuino.h>

DecaDuino decaduino;

uint16_t RX_BUFFER_LENGTH = 127;
uint8_t rxBuffer[127];

const uint16_t LENGTH_MSG_A  = 2;
const uint16_t LENGTH_MSG_B  = 10;
const uint16_t LENGTH_MSG_C  = 10;
uint8_t msgA[LENGTH_MSG_A];
uint8_t msgB[LENGTH_MSG_B];
uint8_t msgC[LENGTH_MSG_C];

uint8_t panId = 30;
uint8_t seqNumber = 0;

// Message A: [0: panId][1: Seq]
// Message B: [0: panId][1: Seq][2-9: tx_{Message A}]
// Message C: [0: panId][1: Seq]2-9: rx_{Message Slave::Tx_t3}]

void buildMsgA()
{
    msgA[0] = panId;
    msgA[1] = seqNumber;
}

void buildMsgB()
{
    msgB[0] = panId;
    msgB[1] = seqNumber;
    decaduino.encodeUint64(decaduino.getLastTxTimestamp(), msgB + 2); // t1
}

void buildMsgC()
{
    msgC[0] = panId;
    msgC[1] = seqNumber;
    decaduino.encodeUint64(decaduino.getLastRxTimestamp(), msgC + 2); // t4
}

void sendFrame(uint8_t *frame, uint16_t frameLength)
{
    decaduino.plmeRxDisableRequest();
    digitalWrite(LED_BUILTIN, HIGH);

    decaduino.pdDataRequest(frame, frameLength);
    while (!decaduino.hasTxSucceeded())
    {
        digitalWrite(LED_BUILTIN, LOW);  delay(50);
        digitalWrite(LED_BUILTIN, HIGH); delay(50);
        decaduino.engine();
    }

    ++seqNumber;

    digitalWrite(LED_BUILTIN, LOW);
}

void masterClapProtocol()
{
    Serial.println("[CLAP] Begin");
  
    seqNumber = 0;

    buildMsgA();
    Serial.print("[SEND] Msg A: ("); Serial.print(seqNumber); Serial.println(")");
    sendFrame(msgA, LENGTH_MSG_A);
    buildMsgB();
    Serial.print("[SEND] Msg B: ("); Serial.print(seqNumber); Serial.print(") "); decaduino.printUint64(decaduino.decodeUint64(msgB + 2)); Serial.println("");
    sendFrame(msgB, LENGTH_MSG_B);

    decaduino.plmeRxEnableRequest();
    bool validFrameReceived = false;
    while (!validFrameReceived)
    {
        decaduino.engine();
        // delay(50);
        if (decaduino.rxFrameAvailable())
        {
            if (rxBuffer[0] == panId) validFrameReceived = true;
            decaduino.plmeRxEnableRequest();
        }

    }
    decaduino.plmeRxDisableRequest();

    delay(500);
    buildMsgC();
    Serial.print("[SEND] Msg C: ("); Serial.print(seqNumber); Serial.print(") "); decaduino.printUint64(decaduino.decodeUint64(msgC + 2)); Serial.println("");
    sendFrame(msgC, LENGTH_MSG_C);

    Serial.println("[CLAP] End");
}

void setup()
{  
    Serial.begin(115200);

    // [DEBUG: begin] Find out if ARDUINO_DWM1001_DEV --------------------------
    #ifdef ARDUINO_DWM1001_DEV
    Serial.println("[DEBUG::Decaduino] ARDUINO_DWM1001_DEV is defined.");
    #else
    Serial.println("[DEBUG::Decaduino] ARDUINO_DWM1001_DEV is not defined.");
    #endif
    // [DEBUG: end] ------------------------------------------------------------
    
    pinMode(LED_BUILTIN, OUTPUT);
    SPI.setSCK(14);

    if (!decaduino.init())
    {
        Serial.println("[Decaduino] Initialization failed.");
        while(true)
        {
            digitalWrite(LED_BUILTIN, LOW);
            delay(50);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
        }
    }

    /*
     * [FEAT: begin] ----------------------------------------------------------
     * Listen to the UWB transceiver and store incoming frames in a buffer,
     * Then, display this message on the serial monitor.
     */
    decaduino.setRxBuffer(rxBuffer, &RX_BUFFER_LENGTH);
    // [FEAT: end] ------------------------------------------------------------
}


void loop()
{
    decaduino.engine();
    masterClapProtocol();

    delay(1000);
}

#include <SPI.h>
#include <DecaDuino.h>

DecaDuino decaduino;

#define INTERLOOP_DELAY 10
#define P               100

uint16_t RX_BUFFER_LENGTH = 10;
uint8_t rxBuffer[10];

uint8_t msgSync[10];
const uint16_t msgSyncLength = 10;

uint8_t panId = 30;
uint8_t seqNumber = 0; 

uint64_t sclk, lclk;

// SYNC: [0: panId][1: Seq][2-9: clk]

void buildSync()
{
    msgSync[0] = panId;
    msgSync[1] = seqNumber;
    decaduino.encodeUint64(sclk, msgSync + 2); // sclk
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

void sispProtocol()
{

    ++lclk;
    ++sclk;

    if (lclk % P == 0)
    {
        Serial.println("--------> ENVOI ");
        buildSync();
        decaduino.plmeRxDisableRequest();
        sendFrame(msgSync, msgSyncLength);
        decaduino.plmeRxEnableRequest();
        Serial.print("lclk ");
        decaduino.printUint64(lclk);
        Serial.println("");
        Serial.print("sclk ");
        decaduino.printUint64(sclk);
        Serial.println("");
    }
    else
    {
        decaduino.plmeRxEnableRequest();
        decaduino.engine();
        if (decaduino.rxFrameAvailable())
        {
            if (rxBuffer[0] == panId) {
                uint64_t rclk = decaduino.decodeUint64(rxBuffer + 2);
                sclk = (sclk + rclk) / 2;
                validFrameReceived = true;
                Serial.println("RECEPTION VALIDE");
            }
            decaduino.plmeRxEnableRequest();
        }

    }
    
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
    
    lclk = 0;
    sclk = 0;
}


void loop()
{
    decaduino.engine();
    sispProtocol();

    delay(INTERLOOP_DELAY);
}

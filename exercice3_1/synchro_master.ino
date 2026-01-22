/*
 * Exercise 3
 * Authors: Amazir Chaouchi, Galatée Marcq
 * 
 * This sketch is a adaptation of the examples called `DecaDuinoSender` and 
 * `DecaDuinoReceiverSniffer`.
 * A copy of these examples can be found in the directory containing this 
 * sketch.
 */

// The following inclusions are mandatory
#include <SPI.h>
#include <DecaDuino.h>

DecaDuino decaduino;

String strMsg;
/*
 * [SOURCE] IEEE 802.15.4 (2024), p.687.
 * The maximum frame length without using the PHR extended field is 127 bits.
 */
#define DATA_FRAME_HEADER_LENGTH 5
#define ACK_FRAME_HEADER_LENGTH 3
#define MAX_FRAME_LENGTH 127
#define MAX_PAYLOAD_LENGTH (MAX_FRAME_LENGTH - DATA_FRAME_HEADER_LENGTH)
uint8_t  rxBuffer[MAX_FRAME_LENGTH];
uint16_t rxBufferLength = MAX_FRAME_LENGTH;
uint8_t  msgBuffer[MAX_PAYLOAD_LENGTH];

// [FEAT: begin] Filtering messages using IEEE 802.15.4 PanID and AddressID ----
uint16_t shortAddress = 2;
uint16_t panId = 30;
// [FEAT: end] -----------------------------------------------------------------

/* [FEAT: begin] DATA + ACK protocol -------------------------------------------
 * Note:
 * Due to limitations, the timeout cannot exceed the maximum representable
 * value of an `uint64_t`, which is 2^32.
 * In order to initialize the variable `timeout` with this value, the value `-1`
 * must be given as right-hand operand of the assignment operator.
 * We use here the property of unsigned arithmetic types in C++, which is that
 * when a negative value is given to such a variable, its bit pattern is 
 * converted to a representable value of the unsigned type.
 * Then, we have:
 * int64_t(-1) = 111....1111 = uint64_t(MAX_UINT64_T)
 * -------------------------------------------------------------------------- */
#define DATA_FRAME_LENGTH MAX_FRAME_LENGTH
#define ACK_FRAME_LENGTH (ACK_FRAME_HEADER_LENGTH + 1)
uint8_t dataFrame[DATA_FRAME_LENGTH];
uint8_t ackFrame[ACK_FRAME_LENGTH];
uint8_t  retries = 3;
uint8_t  lastFrameAcked = 0;
// [FEAT: end] -----------------------------------------------------------------

// [FEAT: begin] timer computation ---------------------------------------------
unsigned long lastTxTimestamp; // millis
unsigned long timeout = 2000;  // millis
// [FEAT: end] -----------------------------------------------------------------

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

    // [FEAT: begin] Filtering messages using IEEE 802.15.4 PanID and AddressID
    decaduino.setShortAddressAndPanId(shortAddress, panId);
    // [DEBUG: begin] Print UWB transceiver shortAddress and PanId -------------
    Serial.print("[DEBUG::Decaduino] shortAddress: "); Serial.println(decaduino.getShortAddress());
    Serial.print("[DEBUG::Decaduino] panId: "); Serial.println(decaduino.getPanId());
    // [DEBUG: end] ------------------------------------------------------------
    // [FEAT: end] -------------------------------------------------------------

    /*
     * [FEAT: begin] ----------------------------------------------------------
     * Listen to the UWB transceiver and store incoming frames in a buffer,
     * Then, display this message on the serial monitor.
     */
    decaduino.setRxBuffer(rxBuffer, &rxBufferLength, uint16_t(MAX_FRAME_LENGTH));
    decaduino.plmeRxEnableRequest();
    // [FEAT: end] ------------------------------------------------------------
}

void loop()
{
    decaduino.engine();

    /* -------------------------------------------------------------------------
     * [FEAT: begin]
     * Read a message from the serial monitor input,
     * then, send this message using the UWB transceiver.
     */
    // readFromSerialAndForward();
    // [FEAT: end] -------------------------------------------------------------

    /* -------------------------------------------------------------------------
     * [FEAT: begin]
     * Listen to the UWB transceiver and store incoming frames in a buffer,
     * Then, display this message on the serial monitor.
     */
    // listenAndDisplayToMonitor();
    // [FEAT: end] -------------------------------------------------------------

    /* -------------------------------------------------------------------------
     * [FEAT: begin]
     * A simple DATA + ACK protocol
     */
    simpleDataAckProcotol(timeout, retries);
    // [FEAT: end] -------------------------------------------------------------

    delay(1000);
}

/*
 * A simple DATA + ACK protocol.
 *
 * Frame layouts:
 * DATA: [0: "0"][1-2: PanId][3: Message number][4: Message length][*: Payload]
 * ACK : [0: "1"][1-2: PanId][3: Number of the acknowledged message]
 *
 * @param timeout: const uint64_t&, the duration between the last time a frame
 *                                  is sent and the moment it becomes outdated
 * @param nbRetriesAllowed: const uint8_t&, the number of attempts allowed
 *                          when no ACK is received of timeout expiration 
 *                          before dropping the frame.
 */
void simpleDataAckProcotol(const uint64_t& timeout, const uint8_t& nbRetriesAllowed)
{
    uint8_t nbRetriesAttempted = 0;

    while(true)
    {
        decaduino.engine();

        /* 
         * Before anything happens, check that the timeout of last sent
         * frame is still valid.
         * If it is not the case, retransmit message.
         *
         * Note:
         * There is no need to use the remainder operator (`%`) here since unsigned
         * arithmetic types automatically wrap-around when there value is greater 
         * than their largest representable value.
         */
        if (millis() > lastTxTimestamp + timeout && nbRetriesAttempted > 0
            && nbRetriesAllowed > nbRetriesAttempted)
        {
            Serial.println("[Decaduino] Timeout expired. Retransmitting...");
            sendFrame(dataFrame);
            ++nbRetriesAttempted;
        }
        else if (nbRetriesAllowed <= nbRetriesAttempted)
        {
            Serial.println("[Decaduino] No retry is allowed. Giving this frame up.");
            nbRetriesAttempted = 0;
        }

        /*
         * If there is data to transmit, and the last message has been
         * successfully acknowledged, send that data.
         * Then, wait for the acknowledgement.
         */ 
        else if (nbRetriesAttempted == 0 && Serial.available())
        {
            readFromSerialAndForward();
            nbRetriesAttempted = 1;
        }

        /*
         * If a message is received:
         * - message: send an ACK 
         * - ACK (implicitly: and timeout is respected), increment the 
         *   lastFrameAcked counter
         */
        if (decaduino.rxFrameAvailable())
        {
            digitalWrite(LED_BUILTIN, HIGH);

            if (decaduino.decodeUint16(rxBuffer + 1) == decaduino.getPanId())
            {
                if (rxBuffer[0] == uint8_t(0)) // DATA frame
                {
                    buildAckFrame(rxBuffer, ackFrame);
                    sendFrame(ackFrame);

                    Serial.println("Receiving frame...");
                    printFrameToSerial(rxBuffer);
                    Serial.println("Sending ACK...");
                    printFrameToSerial(ackFrame);
                }
                else if (rxBuffer[0] == uint8_t(1) // ACK frame
                         && lastFrameAcked + 1 == rxBuffer[1])
                {
                    ++lastFrameAcked;

                    Serial.println("Receiving frame...");
                    printFrameToSerial(ackFrame);
                }
                else Serial.println("Message format not recognized.");
            }

            digitalWrite(LED_BUILTIN, LOW);
        }
    }
}

/*
 * Listen to the UWB transceiver and store incoming frames in a buffer.
 * Then, display this message on the serial monitor.
 */
void listenAndDisplayToMonitor()
{
    if(decaduino.rxFrameAvailable())
    {
        digitalWrite(LED_BUILTIN, HIGH);
        printFrameToSerial(rxBuffer);
        digitalWrite(LED_BUILTIN, LOW);
    }
}

/*
 * If a message is given to the serial input, this function generates a DATA
 * frame and sends it.
 */
void readFromSerialAndForward()
{
    if(Serial.available())
    {
        strMsg = Serial.readStringUntil('\n');
        for (size_t i = 0; i < sizeof msgBuffer && strMsg[i] != '\0'; ++i) msgBuffer[i] = strMsg[i];

        // [FEAT] Send the message ---------------------------------------------
        buildDataFrame(msgBuffer, dataFrame);

        Serial.println("Sending frame...");
        printFrameToSerial(dataFrame);

        sendFrame(dataFrame);
        // [FEAT: end] ---------------------------------------------------------
        
        for (size_t i = 0; i < MAX_PAYLOAD_LENGTH; ++i) msgBuffer[i] = uint8_t('\0');
    }
}

/*
 * Send a frame by using the DecaDuino library and the UWB transceiver.
 * Additionnally, the time the last call to this function is made is stored
 * in the global variable `lastTxTimestamp`, and serve as a reference for
 * timeout mechanisms.
 * 
 * @param frame: uint8_t *, the frame to send
 */
void sendFrame(uint8_t *frame)
{
    decaduino.plmeRxDisableRequest();

    digitalWrite(LED_BUILTIN, HIGH);
    decaduino.pdDataRequest(frame, uint16_t(sizeof frame));
    while (!decaduino.hasTxSucceeded())
    {
        digitalWrite(LED_BUILTIN, LOW);  delay(50);
        digitalWrite(LED_BUILTIN, HIGH); delay(50);
        decaduino.engine();
    }
    digitalWrite(LED_BUILTIN, LOW);

    lastTxTimestamp = millis();
    decaduino.plmeRxEnableRequest();
}

/*
 * Build a frame of type DATA.
 * This frame is represented in memory by an array of MAX_FRAME_LENGTH `uint8_t`s.
 * The content of this array respects the following frame layout:
 * DATA: [0: "0"][1-2: PanId][3: Message number][4: Message length][*: Payload]
 *
 * @param msg: const uint8_t*, the payload of the frame this function is 
                               building
 * @param builtFrame: uint8_t*, the frame this function is building
 */
void buildDataFrame(const uint8_t *msg, uint8_t *builtFrame)
{
    // for (size_t i = 0; i < DATA_FRAME_LENGTH; ++i) builtFrame = 0;

    builtFrame[0] = uint8_t(0);
    decaduino.encodeUint16(decaduino.getPanId(), builtFrame + 1);
    builtFrame[3] = lastFrameAcked + 1;
    builtFrame[4] = 0;
    for (size_t i = 0; msg[i] != '\0'; ++i) ++builtFrame[4];
    for (size_t i = 0; i < MAX_PAYLOAD_LENGTH && msg[i] != '\0'; ++i)
        builtFrame[DATA_FRAME_HEADER_LENGTH + i] = msg[i];

    // [DEBUG: begin] ----------------------------------------------------------
    for (size_t i = 0; i < MAX_FRAME_LENGTH; ++i)
    {
        Serial.print(builtFrame[i]);
        if ((i + 1) % 10 == 0) Serial.println("");
        else Serial.print('\t');
    }
    Serial.println("");
    // [DEBUG: end] ------------------------------------------------------------
}

/*
 * Build a frame of type ACK.
 * This frame is represented in memory by an array of 4 `uint8_t`s.
 * The content of this array respects the following frame layout:
 * ACK : [0: "1"][1-2: PanId][3: Number of the acknowledged message]
 *
 * @param receivedFrame: const uint8_t*, a received frame whose length is
 *                       lower than or equal to MAX_FRAME_LENGTH
 * @param builtFrame: uint8_t*, the frame this function is building
 */
void buildAckFrame(const uint8_t *receivedFrame, uint8_t *builtFrame)
{   
    builtFrame[0] = uint8_t(1);
    builtFrame[1] = receivedFrame[1];
    builtFrame[2] = receivedFrame[2];
    builtFrame[3] = receivedFrame[3];
}

/*
 * Print the content of a frame to the serial monitor.
 * DATA: [0: "0"][1-2: PanId][3: Message number][4: Message length]
 *       [*: Payload]
 *
 * @param frame: const uint8_t*, the frame to print
 */
void printFrameToSerial(const uint8_t *frame)
{
    if (frame[0] == 0) // DATA frame
    {
        Serial.print("[Decaduino::DataFrame] #");
        Serial.print(frame[3]);
        Serial.print(": \"");
        for (size_t i = DATA_FRAME_HEADER_LENGTH;
             i < size_t(DATA_FRAME_HEADER_LENGTH + frame[4]) && frame[i] != '\0';
             ++i)
        {
            Serial.print(char(frame[i]));
        }
        Serial.println("\"");
    }
    else if (frame[0] == 1) // ACK frame
    {
        Serial.print("[Decaduino::AckFrame] ACK #");
        Serial.println(frame[3]);
    }
    else Serial.println("Message format not recognized.");
}

void printFullDebugInfo()
{
    for (size_t i = 0; i < 80; ++i) Serial.print("-");
    Serial.println("");

    Serial.print("[DEBUG::Decaduino] antennaDelay: "); Serial.println(decaduino.getAntennaDelay());
    Serial.print("[DEBUG::Decaduino] channel: "); Serial.println(decaduino.getChannel());
    Serial.print("[DEBUG::Decaduino] channelRaw: "); Serial.println(decaduino.getChannelRaw());
    Serial.print("[DEBUG::Decaduino] euid: "); Serial.println(decaduino.getEuid());
    Serial.print("[DEBUG::Decaduino] lastRxSkew: "); Serial.println(decaduino.getLastRxSkew());
    Serial.print("[DEBUG::Decaduino] lastRxTimestamp: "); Serial.println(decaduino.getLastRxTimestamp());
    Serial.print("[DEBUG::Decaduino] lastTxTimestamp: "); Serial.println(decaduino.getLastTxTimestamp());
    Serial.print("[DEBUG::Decaduino] panId: "); Serial.println(decaduino.getPanId());
    Serial.print("[DEBUG::Decaduino] PHRMode: "); Serial.println(decaduino.getPHRMode());
    Serial.print("[DEBUG::Decaduino] preambleLength: "); Serial.println(decaduino.getPreambleLength());
    Serial.print("[DEBUG::Decaduino] rxPcode: "); Serial.println(decaduino.getRxPcode());
    Serial.print("[DEBUG::Decaduino] rxPrf: "); Serial.println(decaduino.getRxPrf());
    Serial.print("[DEBUG::Decaduino] shortAddress: "); Serial.println(decaduino.getShortAddress());
    Serial.print("[DEBUG::Decaduino] systemTimeCounter: "); Serial.println(decaduino.getSystemTimeCounter());
    Serial.print("[DEBUG::Decaduino] temperature: "); Serial.println(decaduino.getTemperature());
    Serial.print("[DEBUG::Decaduino] temperatureRaw: "); Serial.println(decaduino.getTemperatureRaw());
    Serial.print("[DEBUG::Decaduino] trxStatus: "); Serial.println(decaduino.getTrxStatus());
    Serial.print("[DEBUG::Decaduino] txPcode: "); Serial.println(decaduino.getTxPcode());
    Serial.print("[DEBUG::Decaduino] voltage: "); Serial.println(decaduino.getVoltage());
    Serial.print("[DEBUG::Decaduino] voltageRaw: "); Serial.println(decaduino.getVoltageRaw());

    for (size_t i = 0; i < 80; ++i) Serial.print("-");
    Serial.println("");
}


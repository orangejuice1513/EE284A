/* Sends the CPU temperature via LoRa and listens for repeated reply */
#include <SPI.h>
#include <Arduino.h>
#include <RH_RF95.h>

extern "C" uint8_t temprature_sens_read();

#define RF_FREQUENCY          916000000  // Hz
#define TX_OUTPUT_POWER       20         // dBm
#define LORA_BANDWIDTH        250000     // 250kHz
#define LORA_SPREADING_FACTOR   11        
#define LORA_CODINGRATE       5        // numerator is always 4 
#define LORA_PREAMBLE_LENGTH  16
#define BUFFER_SIZE           (256 - 16) 
#define LORA_SYMBOL_TIMEOUT 0 // Symbols 
#define LORA_FIX_LENGTH_PAYLOAD_ON false 
#define LORA_SYNC_BYTE 0x2B // Meshtastic uses 0x2B


#define RFM95_RST 27   
#define RFM95_CS  33   
#define RFM95_INT A5   

RH_RF95 rf95(RFM95_CS, RFM95_INT);
uint8_t txpacket[BUFFER_SIZE], rxpacket[BUFFER_SIZE];
int16_t rssi = 0, rxSize = 0;

// Write an SPI register and return the read-back value to verify
static uint8_t spiWriteReg(uint8_t addr, uint8_t val) {
    SPISettings cfg(8000000, MSBFIRST, SPI_MODE0);
    SPI.beginTransaction(cfg); digitalWrite(RFM95_CS, LOW);
    SPI.transfer(addr | 0x80); SPI.transfer(val);
    digitalWrite(RFM95_CS, HIGH); SPI.endTransaction();
    SPI.beginTransaction(cfg); digitalWrite(RFM95_CS, LOW);
    SPI.transfer(addr & 0x7F); uint8_t rb = SPI.transfer(0x00);
    digitalWrite(RFM95_CS, HIGH); SPI.endTransaction();
    return rb;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Lab1_Sec5_LoRa.ino V3 916MHz w/chan_hash");

    pinMode (RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    if (!rf95.init()) { Serial.println("RFM95 init failed"); while (1) delay(10); }

    SPI.begin();

    rf95.setFrequency(RF_FREQUENCY / 1e6);  // library expects MHz
    rf95.setTxPower(TX_OUTPUT_POWER, false);
    rf95.setSignalBandwidth(LORA_BANDWIDTH);
    rf95.setCodingRate4(LORA_CODINGRATE);
    rf95.setSpreadingFactor(LORA_SPREADING_FACTOR);
    rf95.setPreambleLength(LORA_PREAMBLE_LENGTH);
    Serial.printf("Sync word 0x39 = 0x%02X\r\n", spiWriteReg(0x39, 0x2B));


    // Meshtastic packet header layout:
    //  Offset  Bytes  Field
    //   0       4     Destination
    //   4       4     Source
    //   8       4     Packet Number
    //  12       1     Flags  (bits 7-5: remaining hops; 4: ACK req; 3: MQTT; 2-0: hop limit)
    //  13       1     chan_hash 
    //  14       1     Next-Hop Address
    //  15       1     Last-Hop Address
    //  16       n     Payload (ASCII)

    const uint32_t destination  = 0xFFFFFFFF; // Broadcast
    const uint32_t source       = 0x06844983; // SU ID
    const uint32_t packetNumber = 0x00000000;
    const uint8_t  flags = 0x63, chan_hash = 0x08, nextHop = 0x00, lastHop = 0x00; // Hop limit 3
    float tempC = (temprature_sens_read() - 32) / 1.8f;

    // Bounds-checked packet builder helpers
    size_t txLen = 0;
    auto append8    = [&](uint8_t v)  { if (txLen < BUFFER_SIZE) txpacket[txLen++] = v; };
    auto append32le = [&](uint32_t v) { for (int i = 0; i < 4; i++) append8((v >> (8*i)) & 0xFF); };
    auto appendStr  = [&](const char *s) { while (*s && txLen < BUFFER_SIZE) txpacket[txLen++] = (uint8_t)*s++; };

    append32le(destination); append32le(source); append32le(packetNumber);
    append8(flags); append8(chan_hash); append8(nextHop); append8(lastHop);  // Useful when we fix chan_hash in the repeater
    // append8(flags); append8(nextHop); append8(lastHop);
    char msg[64];
    snprintf(msg, sizeof(msg), "MCU Temperature = %.2f C", tempC);
    appendStr(msg);

    Serial.printf("\r\nsending packet length %d\r\n", (int)txLen);
    Serial.printf("sending %x", source);
    rf95.send(txpacket, (uint8_t)txLen);
    rf95.waitPacketSent();
}

void loop() {
    if (rf95.available()) {
        uint8_t buf[BUFFER_SIZE], len = sizeof(buf);
        if (rf95.recv(buf, &len)) {
            rxSize = len < BUFFER_SIZE ? len : BUFFER_SIZE - 1;
            memcpy(rxpacket, buf, rxSize);
            rxpacket[rxSize] = '\0';
            rssi = rf95.lastRssi();
            Serial.printf("\r\nreceived packet (len=%d) from source=0x%08X rssi=%d\r\n%s\r\n",
                rxSize, *(uint32_t *)(rxpacket + 4), rssi, (char *)(rxpacket + 16));
        }
    }
}



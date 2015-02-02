#define USE_RADIO
//#define USE_SERIAL    // for debug

#define NODE_MASTER             9
#define NODE_SLAVE              10

//#define PREFER_REPEATER

#define ANALOG_CHANNELS         6       // nr of channels

//#define USE_VCCSTARTCHECK
#define VCC_MIN                 2900 // mV

#define USE_POWEROFF
#define POWEROFF_INTERVALL      (5*60*1000L) // ms

// Config für Modul_Modellbau
#define PIN_RX_PWR              5
#define PIN_TX_PWR              6
#define PIN_LED                 8

/////////////////////  END OF CONFIG  //////////////////////////////////

#include <JeeLib.h>
ISR(WDT_vect)
{
    Sleepy::watchdogEvent();
}   // this must be defined since we're using the watchdog for low-power waiting


#include <RF12sio.h>
RF12    RF12;
#define SEND_MODE               2   // set to 3 if fuses are e=06/h=DE/l=CE, else set to 2
#define HDR_MASTER_MSG          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_DST|0))  // |RF12_HDR_ACK if you want an ACK
#define HDR_MASTER_ACK          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL|RF12_HDR_DST))
#define HDR_SLAVE_MSG           ((NODE_SLAVE&RF12_HDR_MASK)|(0))
#define HDR_SLAVE_ACK           ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL))


#include        <atmel_vcc.h>
ATMEL           atmel = ATMEL();
volatile bool   adcDone;
ISR(ADC_vect)
{
    adcDone = true;
}

struct masterCTRL
{
    uint8_t     com;
    uint8_t     counter;
    uint8_t     digital;
    uint16_t    analog[ANALOG_CHANNELS]; // 8xADC + 1xVCC
    uint16_t    vcc;
    //uint32_t    time;
};
masterCTRL*     msg_received; /// HIER IST EINE ÄNDERUNG in Bezug zur Sendeversion
masterCTRL      msg_valid;
uint8_t         msg_size = sizeof(msg_valid);

uint32_t        time2powerOff;

/////////////////////  PROGRAM:  //////////////////////////////////

uint8_t rf12_handle()
{

    if (rf12_crc != 0)                          return 0;
    if ((rf12_hdr&RF12_HDR_MASK)!=NODE_SLAVE)   return 0;
    if (rf12_len != msg_size)                   return 0;

#ifndef PREFER_REPEATER
    if (msg_received->counter == uint8_t(msg_valid.counter))     return 0;         // already received it
    msg_valid.counter   = msg_received->counter;
#else
    msg_valid.counter   = msg_received->counter+1;
#endif

    // TODO: make shure that there will be no loop when using >1 repeater!

    // strncpy((char*) &msg_valid,(char*) msg_received, msg_size); // doenst work as expected / skips bytes?
    msg_valid.com       = msg_received->com;
    //msg_valid.counter   = msg_received->counter; // Counter-Copy is above
    msg_valid.digital   = msg_received->digital;
    msg_valid.vcc       = msg_received->vcc;
    for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
    {
        msg_valid.analog[ivar] = msg_received->analog[ivar];
    }

    if (msg_received->counter & 1)  digitalWrite(PIN_LED, HIGH);
    else                            digitalWrite(PIN_LED, LOW);

    // begin to transmit received data
    digitalWrite(PIN_TX_PWR, HIGH);
    digitalWrite(PIN_RX_PWR, LOW);
    delay(5); // TODO: must be evaluated

    static boolean received;
    rf12_sleep(RF12_WAKEUP);
    while(!rf12_canSend())
    {
        received |= rf12_recvDone();
    }
    rf12_sendNow(HDR_MASTER_MSG, &msg_valid, msg_size);
    rf12_sendWait(SEND_MODE);
    rf12_sleep(RF12_SLEEP);

    digitalWrite(PIN_TX_PWR, LOW);
    digitalWrite(PIN_RX_PWR, HIGH);

    return 1;
}




void setup()
{

#ifdef USE_SERIAL
    Serial.begin(115200);
#endif // USE_SERIAL

    delay(40);
    rf12_initialize(NODE_SLAVE, RF12_868MHZ, 212); // NodeID, Freq, netGroup
    rf12_initialize(NODE_SLAVE, RF12_868MHZ, 212); // NodeID, Freq, netGroup
    rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
    msg_received = (masterCTRL*) rf12_data;

    // Mod for RFM12BP
    pinMode(        PIN_RX_PWR, OUTPUT);
    digitalWrite(   PIN_RX_PWR, HIGH);

    pinMode(        PIN_TX_PWR, OUTPUT);
    digitalWrite(   PIN_TX_PWR, LOW);

    pinMode(        PIN_LED,    OUTPUT);
    digitalWrite(   PIN_LED,    LOW);

#ifdef USE_VCCSTARTCHECK
    // Safe-VCC-Check
    if (atmel.readVcc() < VCC_MIN)
    {
        while (1) Sleepy::loseSomeTime(1000);
    }
#endif // USE_VCCSTARTCHECK

    time2powerOff = millis() + POWEROFF_INTERVALL;

}



void loop()
{
    uint32_t        loop_time;
    uint8_t         dataValid;

loop_start:

    dataValid     = 0;
    loop_time     = millis();

    if (rf12_recvDone()) dataValid = rf12_handle();

    if (dataValid) time2powerOff = loop_time + POWEROFF_INTERVALL;

#ifdef USE_POWEROFF
    if (loop_time >= time2powerOff)
    {
        digitalWrite(PIN_TX_PWR, LOW);
        digitalWrite(PIN_RX_PWR, LOW); // Fragment of Receive-Debug
        digitalWrite(PIN_LED,    LOW);
        rf12_sleep(RF12_SLEEP);
        Sleepy::loseSomeTime(1000);
        rf12_sleep(RF12_WAKEUP);
        digitalWrite(PIN_RX_PWR, HIGH);
    }
#endif // USE_POWEROFF

    goto loop_start;

}




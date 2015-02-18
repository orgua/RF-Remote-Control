/*
TODO:
- to time-scheduling right ... CanDo Timings! must_send, should_send, could_send
- get vars in functions
- const instead of defines
- transfer code-blocks to libs
- include all in one?

*/

#define USE_RADIO_RFM12
//#define USE_RADIO_RFM95     // TODO: switch to RHReliableDatagram

//#define USE_SERIAL

//#define USE_LATENCYTEST     // Test-Output which triggers a gpio for timing-analysis: high when can_send, low after ADC-sampling

#define NODE_MASTER             9
#define NODE_SLAVE              10

#define MESSAGE_INTERVALL_MIN   20
#define MESSAGE_INTERVALL_MAX   200

#define USE_VCCSTARTCHECK
#define VCC_MIN                 2900 // mV

#define USE_POWEROFF
#define POWEROFF_INTERVALL      (5*60*1000L) // ms

#define ANALOG_CHANNELS         6 // number of channels to transmit
#define ANALOG_JITTER           2
#define ANALOG_SAFEZONE         100

// Config Modul AS2015#A
#define DIGITAL_CHANNELS        0 // how much are used?
#define PIN_DIGITAL_A           0
#define PIN_DIGITAL_B           1
#define PIN_DIGITAL_C           3
#define PIN_DIGITAL_D           4
#define PIN_DIGITAL_E           5   // TODO: 4 INPuT; 4 OUTPUT?
#define PIN_DIGITAL_F           6
#define PIN_DIGITAL_G           7
#define PIN_DIGITAL_H           9

// Config für Modul_Modellbau
#define PIN_RX_PWR              5
#define PIN_TX_PWR              6
#define PIN_LED                 8
#define PIN_LATENCY             3 // INT 1

/////////////////////  END OF CONFIG  //////////////////////////////////

#include <JeeLib.h> // JeeLib RF
ISR(WDT_vect)
{
    Sleepy::watchdogEvent();
}   // this must be defined since we're using the watchdog for low-power waiting

#ifdef USE_RADIO_RFM12
#include <RF12sio.h>
RF12    RF12;
#define SEND_MODE               2   // set to 3 if fuses are e=06/h=DE/l=CE, else set to 2
#define HDR_MASTER_MSG          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_DST|0))
#define HDR_MASTER_ACK          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL|RF12_HDR_DST))
#define HDR_SLAVE_MSG           ((NODE_SLAVE&RF12_HDR_MASK)|(0))
#define HDR_SLAVE_ACK           ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL))
#endif  // USE_RADIO_RFM12

#ifdef USE_RADIO_RFM95
#include <SPI.h>
#include <RH_RF95.h>
RH_RF95 rf95; // Singleton instance of the radio driver
#undef YIELD
#define YIELD Sleepy::loseSomeTime(20);
#endif // USE_RADIO_RFM95

#include <atmel_vcc.h>
ATMEL atmel = ATMEL();
volatile bool adcDone;
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
masterCTRL msg_send;
uint8_t analog_size = ANALOG_CHANNELS * sizeof(msg_send.analog[0]);
uint8_t msg_size    = sizeof(msg_send);

struct masterTEMP
{
    uint8_t     digital;
    uint16_t    analog[ANALOG_CHANNELS]; // previous "long"
};
masterTEMP  msg_old;

uint32_t    time2send_min, time2send_max, time2powerOff;

/////////////////////  PROGRAM:  //////////////////////////////////

void setup()
{

#ifdef USE_SERIAL
    Serial.begin(115200);
#endif

    Sleepy::loseSomeTime(40); // wait for rfm-startup

#ifdef USE_RADIO_RFM12
    //rf12_config();
    rf12_initialize(NODE_MASTER, RF12_868MHZ, 212); // NodeID, Freq, netGroup
    rf12_initialize(NODE_MASTER, RF12_868MHZ, 212);
    rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
    rf12_sleep(RF12_SLEEP);
#endif

#ifdef USE_RADIO_RFM95
    rf95.init();
    rf95.setFrequency(868.0);            // in MHz
    //rf95.setModemConfig(Bw125Cr45Sf128); // Medium Range (lookup _RF95.h)
    rf95.setTxPower(13);                 // 5 ... 23 dBm
    rf95.sleep();
#endif // USE_RADIO_RFM95

    // init vars
    msg_send.counter = 0;
    msg_send.com     = analog_size;

    // timing
    uint32_t  loop_time;
    loop_time     = millis();
    time2send_min = loop_time + MESSAGE_INTERVALL_MIN;
    time2send_max = loop_time + MESSAGE_INTERVALL_MAX;
    time2powerOff = loop_time + POWEROFF_INTERVALL;

    // init Pins
#if DIGITAL_CHANNELS
    pinMode(    PIN_DIGITAL_A,  INPUT);
    pinMode(    PIN_DIGITAL_B,  INPUT);
    pinMode(    PIN_DIGITAL_C,  INPUT);
    pinMode(    PIN_DIGITAL_D,  INPUT);

    pinMode(    PIN_DIGITAL_E,  INPUT);
    pinMode(    PIN_DIGITAL_F,  INPUT);
    pinMode(    PIN_DIGITAL_G,  INPUT);
    pinMode(    PIN_DIGITAL_H,  INPUT);
#endif // DIGITAL_CHANNELS

    for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
    {
        msg_send.analog[ivar]   = analogRead(ivar) + ANALOG_SAFEZONE;
        msg_old.analog[ivar]    = msg_send.analog[ivar];
    }

    // Mod for RFM12BP
    pinMode(        PIN_RX_PWR, OUTPUT);
    digitalWrite(   PIN_RX_PWR, LOW);

    pinMode(        PIN_TX_PWR, OUTPUT);
    digitalWrite(   PIN_TX_PWR, LOW);

    pinMode(        PIN_LED,    OUTPUT);
    digitalWrite(   PIN_LED,    LOW);

#ifdef USE_LATENCYTEST
    pinMode(        PIN_LATENCY,OUTPUT);
    digitalWrite(   PIN_LATENCY,LOW);
#endif // USE_LATENCYTEST

#ifdef USE_VCCSTARTCHECK
    // Safe-VCC-Check
    if (atmel.readVcc() < VCC_MIN)
    {
        while (1)   Sleepy::loseSomeTime(1000);
    }
#endif // USE_VCCSTARTCHECK
}


void loop()
{
    static uint8_t  couldSend, newData, mustSend, shouldSend, led_on;
    uint32_t        loop_time;

loop_start:

    // delay to while
    delay(1);

    // READ DIGITAL
#if DIGITAL_CHANNELS
    if (digitalRead(PIN_DIGITAL_A))   msg_send.digital |=  (1 < 0);
    else                              msg_send.digital &= ~(1 < 0);
    if (digitalRead(PIN_DIGITAL_B))   msg_send.digital |=  (1 < 1);
    else                              msg_send.digital &= ~(1 < 1);
    if (digitalRead(PIN_DIGITAL_C))   msg_send.digital |=  (1 < 2);
    else                              msg_send.digital &= ~(1 < 2);
    if (digitalRead(PIN_DIGITAL_D))   msg_send.digital |=  (1 < 3);
    else                              msg_send.digital &= ~(1 < 3);
    if (digitalRead(PIN_DIGITAL_E))   msg_send.digital |=  (1 < 4);
    else                              msg_send.digital &= ~(1 < 4);
    if (digitalRead(PIN_DIGITAL_F))   msg_send.digital |=  (1 < 5);
    else                              msg_send.digital &= ~(1 < 5);
    if (digitalRead(PIN_DIGITAL_G))   msg_send.digital |=  (1 < 6);
    else                              msg_send.digital &= ~(1 < 6);
    if (digitalRead(PIN_DIGITAL_H))   msg_send.digital |=  (1 < 7);
    else                              msg_send.digital &= ~(1 < 7);
#endif // DIGITAL_CHANNELS


    loop_time     = millis();
    if (loop_time >= time2send_max)     mustSend    = 1;
    if (loop_time >= time2send_min)     couldSend   = 1;
    else                                goto loop_start;

#ifdef USE_LATENCYTEST
    digitalWrite(   PIN_LATENCY, HIGH);
#endif // USE_LATENCYTEST

    // READ ANALOG
    for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
    {
        msg_send.analog[ivar] = analogRead(ivar) + ANALOG_SAFEZONE; // ADD safezone to avoid checking...
        if      (msg_send.analog[ivar] >= msg_old.analog[ivar] + ANALOG_JITTER)   newData = 1;
        else if (msg_send.analog[ivar] <= msg_old.analog[ivar] - ANALOG_JITTER)   newData = 1;
    }

#ifdef USE_LATENCYTEST
    digitalWrite(   PIN_LATENCY, LOW);
#endif // USE_LATENCYTEST

    // Mod for RFM12BP
    if (msg_send.analog[4] > 512 + ANALOG_SAFEZONE)
    {
        digitalWrite(PIN_TX_PWR, HIGH);
        digitalWrite(PIN_RX_PWR, LOW);
        led_on = 1;
    }
    else
    {
        digitalWrite(PIN_TX_PWR, LOW);
        digitalWrite(PIN_RX_PWR, LOW); // Fragment of Receive-Debug
        led_on = 0;
    }

    if (couldSend)    // FLOW: couldSend stays 1 till newData gets in, immediately send out!
    {
        if (newData)
        {
            shouldSend      = 5; // retry rate
            newData         = 0;
            time2powerOff   = loop_time + POWEROFF_INTERVALL;
        }
        if (shouldSend)
        {
            mustSend        = 1;
            couldSend       = 0;
            shouldSend--;
        }
    }

    if (mustSend)
    {

#ifdef USE_RADIO_RFM12
        static boolean received;
        rf12_sleep(RF12_WAKEUP);
        while (!rf12_canSend())
        {
            received |= rf12_recvDone();
        }
        rf12_sendNow(HDR_MASTER_MSG, &msg_send, msg_size);
        rf12_sendWait(SEND_MODE);
        rf12_sleep(RF12_SLEEP);
#endif

#ifdef USE_RADIO_RFM95
        rf95.send((uint8_t*)&msg_send, msg_size);
        rf95.waitPacketSent();
        rf95.sleep();
#endif // USE_RADIO_RFM95

        if (msg_send.counter & led_on)  digitalWrite(PIN_LED, HIGH);
        else                            digitalWrite(PIN_LED, LOW);

        msg_send.counter++;

        // Backup old values
        //strncpy((char*) &msg_old.analog[0],(char*) &msg_send.analog[0], analog_size);
        for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
        {
            msg_old.analog[ivar]    = msg_send.analog[ivar];
        }

        time2send_min   = loop_time + MESSAGE_INTERVALL_MIN;
        time2send_max   = loop_time + MESSAGE_INTERVALL_MAX;

        mustSend        = 0;
        couldSend       = 0;

        // read vcc and store it for next time
        msg_send.vcc    = atmel.readVcc();

#ifdef USE_SERIAL
        Serial.print(msg_send.counter);
        Serial.print(" : ");
        for (byte ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
        {
            Serial.print(msg_send.analog[ivar]);
            Serial.print(" : ");
        }
        Serial.println(msg_send.vcc);
        Serial.print(" : ");
        Serial.println(msg_send.digital, BIN);
        Serial.println("");
#endif
    }
    //else    Sleepy::loseSomeTime(3);

#ifdef USE_POWEROFF
    if (loop_time >= time2powerOff)
    {
        digitalWrite(PIN_TX_PWR, LOW);
        digitalWrite(PIN_RX_PWR, LOW); // Fragment of Receive-Debug
        digitalWrite(PIN_LED, 	 LOW);
        rf12_sleep(RF12_SLEEP);
        Sleepy::loseSomeTime(1000);
    }
#endif // USE_POWEROFF

    goto loop_start;
}

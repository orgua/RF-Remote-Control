/*  DOC:

- Connection to CarreraControler:
   B --> GND (PotiPort 3)
   R --> VCC (Potiport 4 PC5)
   G --> SWITCH (PotiPort 2 PC4)
   Y --> POTI   (PotiPort 5 PC3, PD3, PD1, PB1)
   Coincell (+) to Potiport 1
   Short the Resistor
   
- Energyconsumption:

    wake:           10ms*500uA + 1ms*800uA + 50us*1900uA ) * 3V --> 17.7 uWs
    transmit:       wake + 2ms*800uA + 5ms*1800uA ) * 3V        --> 49.5 uWs

    active-MIN : 2*w + 2*t   -->  134 uWs
    active-MAX : (1/0.032)*t --> 1546 uWs
    sleep      : 1*w         -->   18 uWs

    CR2025 : 150 mAh * 3V --> 1620 Ws
    
    2015-03: new Fusesettings - EXT 0xFD, HIGH 0xDF, LOW 0xD2
    - use internal RC-Oszi / 8MHz
    - BOD 2V7
*/

#define USE_RADIO
//#define USE_SERIAL
//#define USE_LED

#define NODE_MASTER             13  // MMM = 11, III = 13
#define NODE_SLAVE              (NODE_MASTER+1)

#define MESSAGE_INTERVALL_MIN   (16<<2) // 64
#define MESSAGE_INTERVALL_MAX   1000 // (16<<5)=512

//#define USE_VCCSTARTCHECK
#define VCC_MIN                 2500 // mV

#define USE_POWEROFF
#define POWEROFF_INTERVALL      (20*1000L) // ms

#define ANALOG_CHANNELS         1 // number of channels to transmit
#define ANALOG_JITTER           3
#define ANALOG_SAFEZONE         100
#define ANALOG_DEADZONE         30      // for ignoring small drifts (around zero)

// Config for module ArduNode
#define PIN_VCC                 A5
#define PIN_SWITCH              A4
#define PIN_SWITCH_ANALOG       4
#define PIN_POTI                A3
#define PIN_POTI_ANALOG         3
#define PIN_POTI_B              3   // internal connection ....
#define PIN_POTI_C              1   // internal connection ....
#define PIN_POTI_D              9   // internal connection ....
#define PIN_LED                 8

// Config Modul AS2015#A
#define DIGITAL_CHANNELS        2 // how much are used?
#define PIN_DIGITAL_A           PIN_SWITCH
#define PIN_DIGITAL_B           PIN_POTI


/////////////////////  END OF CONFIG  //////////////////////////////////

#include <JeeLib.h> // JeeLib RF
static volatile uint8_t watchdogCounter;
ISR(WDT_vect)
{
    ++watchdogCounter;
}   // this must be defined since we're using the watchdog for low-power waiting

#ifdef USE_RADIO
#include <RF12sio.h>
RF12    RF12;
#define SEND_MODE               2   // set to 3 if fuses are e=06/h=DE/l=CE, else set to 2
#define HDR_MASTER_MSG          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_DST|0))
#define HDR_MASTER_ACK          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL|RF12_HDR_DST))
#define HDR_SLAVE_MSG           ((NODE_SLAVE&RF12_HDR_MASK)|(0))
#define HDR_SLAVE_ACK           ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL))
#endif  // USE_RADIO

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
    uint16_t    analog[ANALOG_CHANNELS]; // 8xADC
    uint16_t    vcc;
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

uint8_t loseDirectTime (uint16_t msecs)
{
    uint16_t msleft = msecs;
    // only slow down for periods longer than the watchdog granularity
    if (msleft <= 16) return 0;

    uint8_t wdp = 0; // wdp 0..9 corresponds to roughly 16..8192 ms
    // calc wdp as log2(msleft/16), i.e. loop & inc while next value is ok
    for (uint16_t m = msleft; m >= 32; m >>= 1)  if (++wdp >= 9) break;

    Sleepy::watchdogInterrupts(wdp);
    Sleepy::powerDown();
    Sleepy::watchdogInterrupts(-1); // off

    // when interrupted, our best guess is that half the time has passed
    if (watchdogCounter == 0)   msleft -= 8 << wdp;
    else                        msleft -= 8 << (wdp+1);

    // adjust the milli ticks, since we will have missed several
#if defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny85__) || defined (__AVR_ATtiny44__) || defined (__AVR_ATtiny45__)
    extern volatile unsigned long millis_timer_millis;
    millis_timer_millis += msecs - msleft;
#else
    extern volatile unsigned long timer0_millis;
    timer0_millis += msecs - msleft;
#endif
    return 1; // true if we lost approx the time planned
}



void setup()
{

#ifdef USE_RADIO
    Sleepy::loseSomeTime(40);
    rf12_initialize(NODE_MASTER, RF12_868MHZ, 212); // NodeID, Freq, netGroup
    rf12_initialize(NODE_MASTER, RF12_868MHZ, 212);
    rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
    rf12_sleep(RF12_SLEEP);
#endif

    // init vars
    msg_send.counter = 0;
    msg_send.com     = analog_size;

    // timing
    uint32_t  loop_time;
    loop_time     = millis();
    time2send_min = loop_time + MESSAGE_INTERVALL_MIN;
    time2send_max = loop_time + MESSAGE_INTERVALL_MAX;
    time2powerOff = loop_time; // instantly go to poweroff

    pinMode(        PIN_LED,    OUTPUT);    // help to see if node started up right (short blink)
    digitalWrite(   PIN_LED,    HIGH);

    // init Pins
    msg_send.analog[0]  = analogRead(PIN_POTI_ANALOG) + ANALOG_SAFEZONE;
    msg_old.analog[0]   = msg_send.analog[0];
    msg_send.vcc        = atmel.readVcc();

    // Attempt to save power --> not success so far
    DIDR0 = 0xFF & ~((1<<PIN_SWITCH_ANALOG) | (1<<PIN_POTI_ANALOG));
    PRR |= (1<<7); // disable TWI
    PRR |= (1<<6); // disable TIM2
    PRR |= (1<<3); // disable TIM1
    PRR |= (1<<1); // disable UART0
    ADCSRA &= ~ bit(ADEN); // disable ADC (first Step)
    PRR |= (1<<0); // disable ADC

    pinMode(PIN_VCC,      OUTPUT);
    digitalWrite(PIN_VCC, LOW);

    pinMode(PIN_POTI,     INPUT);
    pinMode(PIN_POTI_B,   INPUT);
    pinMode(PIN_POTI_C,   INPUT);
    pinMode(PIN_POTI_D,   INPUT);

    pinMode(PIN_SWITCH,   INPUT_PULLUP);

#ifndef USE_LED
    digitalWrite(   PIN_LED,    LOW);
    pinMode(        PIN_LED,    INPUT);
#endif // USE_LED


#ifdef USE_VCCSTARTCHECK
    // Safe-VCC-Check
    if (atmel.readVcc() < VCC_MIN)
    {
        while (1)
        {
            Sleepy::loseSomeTime(1000);
        }
    }
#endif // USE_VCCSTARTCHECK
}


void loop()
{
    static uint8_t  couldSend, newData, mustSend, shouldSend;
    uint32_t        loop_time;
    uint16_t        sleep_intervall = MESSAGE_INTERVALL_MIN;

loop_start:

    loop_time     = millis();
    if (loop_time >= time2send_max)     mustSend    = 1;
    if (loop_time >= time2send_min)     couldSend   = 1;

    // READ DATA

    PRR &= ~(1<<0); // enable ADC
    ADCSRA |= bit(ADEN); // enable ADC (last Step)
    digitalWrite(PIN_VCC, HIGH);
    msg_send.digital    = digitalRead(PIN_SWITCH);
    if      (msg_send.digital != msg_old.digital)                       newData = 1;

    msg_send.analog[0]  = analogRead(PIN_POTI_ANALOG) + ANALOG_SAFEZONE; // ADD safezone to avoid checking...
    if      (msg_send.analog[0] >= msg_old.analog[0] + ANALOG_JITTER)   newData = 1;
    else if (msg_send.analog[0] <= msg_old.analog[0] - ANALOG_JITTER)   newData = 1;
    else if (msg_send.analog[0] > ANALOG_SAFEZONE + 50)                 time2powerOff   = loop_time + POWEROFF_INTERVALL;
    digitalWrite(PIN_VCC, LOW);
    ADCSRA &= ~ bit(ADEN); // disable ADC (first Step)
    PRR |= (1<<0); // disable ADC

    if (couldSend)    // FLOW: couldSend stays 1 till newData gets in, immediatly send out!
    {
        if (newData)
        {
            shouldSend      = 3; // retry rate
            newData         = 0;
            time2powerOff   = loop_time + POWEROFF_INTERVALL;
            sleep_intervall = MESSAGE_INTERVALL_MIN;
        }

        if (mustSend) // slowly increase sleepcycles
        {
            if (sleep_intervall < 256) sleep_intervall = sleep_intervall<<1;
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

#ifdef USE_RADIO
        static boolean  received;
        uint8_t         send_try = 250;

        rf12_sleep(RF12_WAKEUP);
        while ((!rf12_canSend()) && send_try)
        {
            received |= rf12_recvDone();
            send_try--;
        }
        if (send_try)
        {
            rf12_sendNow(HDR_MASTER_MSG, &msg_send, msg_size);
            rf12_sendWait(SEND_MODE);
            rf12_sleep(RF12_SLEEP);
#endif

#ifdef USE_LED
            if (msg_send.counter & 1)   digitalWrite(PIN_LED, HIGH);
            else                        digitalWrite(PIN_LED, LOW);
#endif // USE_LED

            msg_send.counter++;

            // Backup old values
            msg_old.analog[0]   = msg_send.analog[0];
            msg_old.digital     = msg_send.digital;

            time2send_min       = loop_time + MESSAGE_INTERVALL_MIN;
            time2send_max       = loop_time + MESSAGE_INTERVALL_MAX;

            mustSend            = 0;
            couldSend           = 0;

        }
        else   rf12_sleep(RF12_SLEEP);
    }


#ifdef USE_POWEROFF
    if (loop_time >= time2powerOff)
    {
        digitalWrite(PIN_VCC, HIGH);
        while(!digitalRead(PIN_POTI))
        {
            digitalWrite(PIN_VCC, LOW);
            Sleepy::watchdogInterrupts(6); // 0..9 corresponds to roughly 16..8192 ms
            Sleepy::powerDown();
            Sleepy::watchdogInterrupts(-1);
            digitalWrite(PIN_VCC, HIGH);
        }
        // read vcc and store it for this wake-period
        PRR &= ~(1<<0); // enable ADC
        ADCSRA |= bit(ADEN); // enable ADC (last Step)
        msg_send.vcc        = atmel.readVcc();
        // wait till device is unpressed
        while(analogRead(PIN_POTI_ANALOG)>ANALOG_DEADZONE)
        {
            digitalWrite(PIN_VCC, LOW);
            Sleepy::watchdogInterrupts(6); // 0..9 corresponds to roughly 16..8192 ms
            Sleepy::powerDown();
            Sleepy::watchdogInterrupts(-1);
            digitalWrite(PIN_VCC, HIGH);
        }
        time2powerOff   = millis() + POWEROFF_INTERVALL;
    }
    else    loseDirectTime(sleep_intervall);
#endif // USE_POWEROFF

    goto loop_start;
}

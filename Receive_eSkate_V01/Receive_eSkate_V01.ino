#define USE_RADIO
//#define USE_SERIAL    // for debug
#define USE_PWM
//#define USE_BRAKES

#define NODE_MASTER             13  // MMM = 11, III = 13
#define NODE_SLAVE              (NODE_MASTER+1)

#define ANALOG_CHANNELS         1       // nr of channels
#define ANALOG_JITTER           2       // small changes won't trigger fast sending
#define ANALOG_SAFEZONE         100     // better use of uint16 (move away from 0)
#define ANALOG_DEADZONE         30      // for ignoring small drifts (around zero)

#ifdef USE_BRAKES
#define STDVALUE                1640
#else
#define STDVALUE                1120
#endif // USE_BRAKES

#define USE_VCCSTARTCHECK
#define VCC_MIN                 2500 // mV

#define PIN_LED                 8
#define PIN_PWM                 A3

#ifdef USE_SERIAL
#define CONTROL_INTERVALL_MIN   50      // for debug
#define CONTROL_INTERVALL_MAX   500
#else
#define CONTROL_INTERVALL_MIN   10      // 7-10 produziert stottern
#define CONTROL_INTERVALL_MAX   50     // test: 20/50
#endif // USE_SERIAL

#define ERROR_TIME_MS           3000L //
#define ERROR_THRESHOLD         (ERROR_TIME_MS/CONTROL_INTERVALL_MAX) // 60

/////////////////////  END OF CONFIG  //////////////////////////////////

#include <JeeLib.h>
ISR(WDT_vect)
{
    Sleepy::watchdogEvent();
}   // this must be defined since we're using the watchdog for low-power waiting

#ifdef USE_RADIO
#include <RF12sio.h>
RF12    RF12;
#define SEND_MODE               2   // set to 3 if fuses are e=06/h=DE/l=CE, else set to 2
#define HDR_MASTER_MSG          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_DST|0))  // |RF12_HDR_ACK if you want an ACK
#define HDR_MASTER_ACK          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL|RF12_HDR_DST))
#define HDR_SLAVE_MSG           ((NODE_SLAVE&RF12_HDR_MASK)|(0))
#define HDR_SLAVE_ACK           ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL))
#endif // USE_RADIO

#include <atmel_vcc.h>
ATMEL atmel = ATMEL();
volatile bool adcDone;
ISR(ADC_vect)
{
    adcDone = true;
}

#ifdef      USE_PWM
#include    <Servo.h>
#define     SERVOS      1
#define     SERVOMIN    800
#define     SERVOMAX    2400
Servo       servoA;
#endif      // USE_PWM


struct masterCTRL
{
    uint8_t     com;
    uint8_t     counter;
    uint8_t     digital;
    uint16_t    analog[ANALOG_CHANNELS]; // 8xADC + 1xVCC
    uint16_t    vcc;
};
masterCTRL* msg_received;
masterCTRL  msg_valid;
uint8_t     msg_size = sizeof(msg_valid);

uint16_t    receive_errors; // use for QoS

/////////////////////  PROGRAM:  //////////////////////////////////

uint8_t rf12_handle()
{

    if (rf12_crc != 0)                          return 0;
    if ((rf12_hdr&RF12_HDR_MASK)!=NODE_SLAVE)   return 0;
    if (rf12_len != msg_size)                   return 0;

#ifdef USE_VCCSTARTCHECK
    if (msg_received->vcc < VCC_MIN)            return 0;
#endif // USE_VCCSTARTCHECK

    int16_t puffer[ANALOG_CHANNELS];

    for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
    {
        puffer[ivar] = msg_received->analog[ivar];
        if (puffer[ivar] > 1024 + ANALOG_SAFEZONE)  return 0;
        if (puffer[ivar] < ANALOG_SAFEZONE)         return 0;
    }

    if (msg_received->counter == uint8_t(msg_valid.counter))     return 0;         // already received it
    if (msg_received->counter != uint8_t(msg_valid.counter + 1)) receive_errors++; // for statistics / later

    msg_valid.com       = msg_received->com;
    msg_valid.counter   = msg_received->counter;
    msg_valid.digital   = msg_received->digital;
    msg_valid.vcc       = msg_received->vcc;
    for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
    {
        msg_valid.analog[ivar] = puffer[ivar] - ANALOG_SAFEZONE;
    }

    if (msg_received->counter & 1)  digitalWrite(PIN_LED, HIGH);
    else                            digitalWrite(PIN_LED, LOW);

    return 1;
}

void setup()
{

#ifdef USE_SERIAL
    Serial.begin(115200);
#endif // USE_SERIAL

#ifdef USE_RADIO
    Sleepy::loseSomeTime(40);
    rf12_initialize(NODE_SLAVE, RF12_868MHZ, 212); // NodeID, Freq, netGroup
    rf12_initialize(NODE_SLAVE, RF12_868MHZ, 212); // NodeID, Freq, netGroup
    rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
    msg_received = (masterCTRL*) rf12_data;
#endif // USE_RADIO

    servoA.attach(PIN_PWM,SERVOMIN,SERVOMAX);

    for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
    {
        msg_valid.analog[ivar] = 0;
    }
    msg_valid.digital = 0;
    msg_valid.counter = 255; // correct errorrate-values

    pinMode(        PIN_LED, OUTPUT);
    digitalWrite(   PIN_LED, HIGH);
}


void loop()
{
    uint32_t        time2ctrl_min = 0, time2ctrl_max = 0;
    static uint16_t errorCounter = 0;
    uint16_t        servo, servoPT;
    uint8_t         been_zero, brake_mode;

loop_start:

    uint8_t         dataValid = 0;

    if (rf12_recvDone())
    {
        dataValid = rf12_handle();

        // Send out an ACK and put the actual PWM_Value inside
        if (RF12_WANTS_ACK)
        {
            rf12_sendStart(HDR_SLAVE_ACK, 0, 0);
            rf12_sendWait(SEND_MODE);
        }
    }

    static uint8_t  mustCTRL = 0, canCTRL = 0;
    static uint32_t loop_time;

    loop_time     = millis();
    if      (loop_time >= time2ctrl_max)     mustCTRL = 1;
    else if (loop_time >= time2ctrl_min)     canCTRL  = 1;

    if (canCTRL && dataValid)
    {
        mustCTRL        = 1;
        canCTRL         = 0;
        dataValid       = 0;
        errorCounter    = 0;
    }

    if (mustCTRL)
    {
        if (errorCounter > ERROR_THRESHOLD)
        {
            // place safe value
            msg_valid.digital       = 255; // turn on brakes
            msg_valid.analog[0]     = 1; // switch to stable Mode!
            servo                   = 1;

            msg_valid.counter       = 255; // correct errorrate-values
        }
        else    errorCounter++; // will reset if valid paket comes in

        servoPT = (servoPT + msg_valid.analog[0])>>1; // PT-Smoothing

        if (servoPT < ANALOG_DEADZONE)
        {
            // power-value is below the threshold
            been_zero   = 1;
            // set the standard servo-output
            servoPT     = 0;
            servo       = 0;
            // decide if we want to break or accelerate
            if (msg_valid.digital)  brake_mode = 0;
            else                    brake_mode = 1;

        }
        else
        {
            // power-value is above the threshold
            if (been_zero==1)   been_zero = 0;
            // The Maik-Magic happens here!
#ifdef USE_BRAKES
            if (brake_mode==1)  servo = 1460 - (servoPT>>1);
            else                servo = 1650 + (servoPT>>2) + (servoPT>>1);
#else
            if (brake_mode==1)  servo = 0;
            else                servo = STDVALUE + servoPT - ANALOG_DEADZONE;
#endif

        }

        servoA.writeMicroseconds(servo); // TODO: has to be between 1100 and 1900, now: 1250

        time2ctrl_min   = loop_time + CONTROL_INTERVALL_MIN;
        time2ctrl_max   = loop_time + CONTROL_INTERVALL_MAX;
        mustCTRL        = 0;

    }

    goto loop_start;

}




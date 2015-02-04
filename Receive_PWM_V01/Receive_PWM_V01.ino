#define USE_RADIO
//#define USE_SERIAL    // for debug
#define USE_PWM

#define NODE_MASTER             9
#define NODE_SLAVE              10

#define ANALOG_CHANNELS         6       // nr of channels
#define ANALOG_JITTER           2       // small changes won't trigger fast sending
#define ANALOG_SAFEZONE         100     // better use of uint16 (move away from 0)
#define ANALOG_DEADZONE         8      // for ignoring small drifts (around zero)
#define STDVALUE                512     // middle of [0...1023]

#define PIN_LED                 8

#ifdef USE_SERIAL
#define CONTROL_INTERVALL_MIN   50      // for debug
#define CONTROL_INTERVALL_MAX   500
#else
#define CONTROL_INTERVALL_MIN   10      // 7-10 produziert stottern
#define CONTROL_INTERVALL_MAX   50     // test: 20/50
#endif // USE_SERIAL

#define ERROR_TIME_MS           4000L
#define ERROR_THRESHOLD         (ERROR_TIME_MS/CONTROL_INTERVALL_MAX)
#define ERROR_SCALEFAK          20  // to get slower fallrate / has to be lower than threshold
#define ERROR_FALLRATE          ceil(200.0 * ERROR_SCALEFAK * CONTROL_INTERVALL_MAX / 30000.0) // lower Throttle 200n in 30sec
#define ERROR_THROTTLE_MIN      350     // drop till this point

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
#define     SERVOS      8
#define     SERVOMIN    500
#define     SERVOMAX    2000
Servo       servoA;
Servo       servoB;
Servo       servoC;
Servo       servoD;

Servo       servoE;
Servo       servoF;
Servo       servoG;
Servo       servoH;
#endif      // USE_PWM


#include <atmel_eFunction.h>
eFunction eFkt;

struct masterCTRL
{
    uint8_t     com;
    uint8_t     counter;
    uint8_t     digital;
    uint16_t    analog[ANALOG_CHANNELS]; // 8xADC + 1xVCC
    uint16_t    vcc;
    //uint32_t    time;
};
masterCTRL* msg_received; /// HIER IST EINE ÄNDERUNG in Bezug zur Sendeversion
masterCTRL  msg_valid;
uint8_t     msg_size = sizeof(msg_valid);

int16_t     servo[ANALOG_CHANNELS], backup_throttle;
uint16_t    receive_errors; // use for QoS

/////////////////////  PROGRAM:  //////////////////////////////////

uint8_t rf12_handle()
{

    if (rf12_crc != 0)                          return 0;
    if ((rf12_hdr&RF12_HDR_MASK)!=NODE_SLAVE)   return 0;
    if (rf12_len != msg_size)                   return 0;

    int16_t puffer[ANALOG_CHANNELS];

    for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
    {
        puffer[ivar] = msg_received->analog[ivar];
        if (puffer[ivar] > 1024 + ANALOG_SAFEZONE)  return 0;
        if (puffer[ivar] < ANALOG_SAFEZONE)         return 0;
    }

    if (msg_received->counter == uint8_t(msg_valid.counter))     return 0;         // already received it
    if (msg_received->counter != uint8_t(msg_valid.counter + 1)) receive_errors++; // for statistics / later

    // strncpy((char*) &msg_valid,(char*) msg_received, msg_size); // doenst work as expected / skips bytes?
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

    //rssi = rf12_control();

#ifdef USE_SERIAL5
    for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
    {
        Serial.print(msg_received->analog[ivar]);
        Serial.print(" : ");
    }
    Serial.print(msg_received->digital, BIN);
    Serial.print(" : ");
    Serial.print(msg_received->com);
    Serial.print(" : ");
    Serial.print(msg_received->vcc);
    Serial.print(" : ");
    Serial.print(msg_received->counter);
    Serial.println("");
#endif // USE_SERIAL

    return 1;
}


uint32_t    time2ctrl_min, time2ctrl_max;

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

    servoA.attach(0,SERVOMIN,SERVOMAX);
    servoB.attach(1,SERVOMIN,SERVOMAX);
    servoC.attach(3,SERVOMIN,SERVOMAX);
    servoD.attach(4,SERVOMIN,SERVOMAX);

    servoE.attach(5,SERVOMIN,SERVOMAX);
    servoF.attach(6,SERVOMIN,SERVOMAX);
    servoG.attach(7,SERVOMIN,SERVOMAX);
    servoH.attach(9,SERVOMIN,SERVOMAX);

    for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
    {
        msg_valid.analog[ivar] = STDVALUE;
    }
    msg_valid.counter = 255; // correct errorrate-values

    pinMode(        PIN_LED, OUTPUT);
    digitalWrite(   PIN_LED, HIGH);

    eFkt.init(INTERVAL,1.6);
    eFkt.set_zeropoint(STDVALUE);

    uint32_t  loop_time;
    loop_time     = millis();
    time2ctrl_min = loop_time + CONTROL_INTERVALL_MIN;
    time2ctrl_max = loop_time + CONTROL_INTERVALL_MAX;
}



void loop()
{
loop_start:

    uint8_t         dataValid = 0;
    static uint8_t  errorDetect;
    static uint16_t errorCounter = 0;

    if (rf12_recvDone())
    {
        dataValid = rf12_handle();

        // Send out an ACK and put the actual PWM_Value inside
        if (RF12_WANTS_ACK)
        {
            //ack_send.servo = servo;
            //ack_send.akku = akku;
            //rf12_sendStart(HDR_ACK, &ack_send, sizeof ack_send);
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
        errorDetect     = 0;
    }

    if (mustCTRL)
    {
        if (errorCounter > ERROR_THRESHOLD)
        {

            errorCounter           -= ERROR_SCALEFAK;

            // place safe value
            msg_valid.analog[0]     = STDVALUE;
            msg_valid.analog[1]     = STDVALUE;

            if (errorDetect == 0)
            {
                errorDetect         = 1;
                msg_valid.analog[2] = STDVALUE + 50;
            }

            if (msg_valid.analog[2] > ERROR_THROTTLE_MIN)   msg_valid.analog[2] -= ERROR_FALLRATE;
            else                                            msg_valid.analog[2]  = ERROR_THROTTLE_MIN;

            msg_valid.analog[3]     = STDVALUE;
            msg_valid.analog[4]     = STDVALUE;
            msg_valid.analog[5]     = 1023; // switch to stable Mode!

            msg_valid.counter       = 255; // correct errorrate-values
        }
        else    errorCounter++; // will reset if valid paket comes in


        for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
        {
            if      (msg_valid.analog[ivar] > STDVALUE + ANALOG_DEADZONE)   servo[ivar] = msg_valid.analog[ivar] - ANALOG_DEADZONE;
            else if (msg_valid.analog[ivar] < STDVALUE - ANALOG_DEADZONE)   servo[ivar] = msg_valid.analog[ivar] + ANALOG_DEADZONE;
            else                                                            servo[ivar] = STDVALUE;
        }

        // Normale Belegung:
        //              Vertikal    Horizontal
        // Links:   |   Throttle, - Rudder
        // Rechts:  |   Elevator, - Aileron

        //  CH0 R H RechtsPlus
        //  CH1 R V ObenMinus
        //  CH2 L V ObenPlus
        //  CH3 L H RechtsMinus

        servoA.writeMicroseconds(1000 + eFkt.get(servo[0])); // TODO: has to be between 1100 and 1900, now: 1250
        servoB.writeMicroseconds(1000 + eFkt.get(servo[1]));
        servoC.writeMicroseconds(1000 + servo[2]);
        servoD.writeMicroseconds(1000 + eFkt.get(servo[3]));

        servoE.writeMicroseconds(1000 + servo[5]);
        servoF.writeMicroseconds(1000 + 512);
        servoG.writeMicroseconds(1000 + 512);
        servoH.writeMicroseconds(1000 + 512);

        time2ctrl_min   = loop_time + CONTROL_INTERVALL_MIN;
        time2ctrl_max   = loop_time + CONTROL_INTERVALL_MAX;
        mustCTRL        = 0;

#ifdef USE_SERIAL
        Serial.print(msg_size);
        Serial.print(" :: ");
        for (uint8_t ivar = 0; ivar < 5; ivar++)
        {
            Serial.print(spektrumCH[ivar]);
            Serial.print(" : ");
        }
        Serial.print(msg_valid.digital, BIN);
        Serial.print(" : ");
        Serial.print(msg_valid.com);
        Serial.print(" : ");
        Serial.print(msg_valid.vcc);
        Serial.print(" : ");
        Serial.print(msg_valid.counter);
        Serial.print(" : E");
        Serial.print(receive_errors);
        Serial.println("");

#endif // USE_SERIAL

    }

    goto loop_start;

}




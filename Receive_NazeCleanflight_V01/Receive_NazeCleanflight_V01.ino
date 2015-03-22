#define USE_RADIO_RFM12
//#define USE_RADIO_RFM95
#define USE_SPEKTRUM    // http://blog.kwarf.com/2013/08/a-look-at-the-spektrum-satellite-protocol/


#define NODE_MASTER             9
#define NODE_SLAVE              10

#define ANALOG_CHANNELS         6       // nr of channels
#define ANALOG_JITTER           2       // small changes won't trigger fast sending
#define ANALOG_SAFEZONE         100     // better use of uint16 (move away from 0)
#define ANALOG_DEADZONE         8       // for ignoring small drifts (around zero)
#define STDVALUE                512     // middle of [0...1023]

#define PIN_LED                 8
#define PIN_LATENCY             3 // INT 1

#define CONTROL_INTERVALL_MIN   (8L)
#define CONTROL_INTERVALL_MAX   (16L)
// cleanflight is ok with 5ms distance! https://github.com/cleanflight/cleanflight/blob/master/src/main/rx/spektrum.c
// 16byte Frame takes about 1.5ms 

#define ERROR_TIME_MS           5000L
#define ERROR_THRESHOLD         ceil(ERROR_TIME_MS/CONTROL_INTERVALL_MAX)
#define ERROR_SCALEFAK          80L  // to get slower fallrate / has to be lower than threshold
#define ERROR_FALLRATE          ceil(200.0 * ERROR_SCALEFAK * CONTROL_INTERVALL_MAX / 30000.0) // lower Throttle 200n in 30sec
#define ERROR_THROTTLE_MIN      350     // drop till this point

/////////////////////  END OF CONFIG  //////////////////////////////////

#include <JeeLib.h>
ISR(WDT_vect)
{
    Sleepy::watchdogEvent();
}   // this must be defined since we're using the watchdog for low-power waiting

#ifdef USE_RADIO_RFM12
#include <RF12sio.h>
RF12    RF12;
#define SEND_MODE               2   // set to 3 if fuses are e=06/h=DE/l=CE, else set to 2
#define HDR_MASTER_MSG          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_DST|0))  // |RF12_HDR_ACK if you want an ACK
#define HDR_MASTER_ACK          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL|RF12_HDR_DST))
#define HDR_SLAVE_MSG           ((NODE_SLAVE&RF12_HDR_MASK)|(0))
#define HDR_SLAVE_ACK           ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL))
#endif // USE_RADIO_RFM12

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

#ifdef USE_SPEKTRUM
#include <atmel_spektrumSerial.h>
#endif // USE_SPEKTRUM

#include <PowerSaver.h>
PowerSaver ps;

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
masterCTRL  msg_valid, msg_rfm95;
uint8_t     msg_size = sizeof(msg_valid);

int16_t     servo[ANALOG_CHANNELS], backup_throttle;
uint16_t    receive_errors; // use for QoS

/////////////////////  PROGRAM:  //////////////////////////////////

uint8_t rfm_handle()
{

#ifdef USE_RADIO_RFM12
    if (rf12_crc != 0)                          return 0;
    if ((rf12_hdr&RF12_HDR_MASK)!=NODE_SLAVE)   return 0;
    if (rf12_len != msg_size)                   return 0;
#endif // USE_RADIO_RFM12

#ifdef USE_RADIO_RFM95
    uint8_t rf95_len = RH_RF95_MAX_MESSAGE_LEN;
    if (!rf95.recv((uint8_t*)&msg_rfm95,&rf95_len))     return 0;
    if (rf95_len != msg_size)                           return 0;
    // TODO check CRC
#endif // USE_RADIO_RFM95

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
    return 1;
}



void setup()
{

    ps.turnOffTWI();
    ps.turnOffTimer1();
    ps.turnOffTimer2();
    ps.turnOffDigitalInput();  
    
#ifdef USE_SPEKTRUM
    spektrumSerial_init();
    //#define UART_BAUD_RATE 104200   // hacker bei ROLL
    #define UART_BAUD_RATE 107500   // hacker bei ROLL
    #define UART_BAUD_SELECT ((F_CPU/UART_BAUD_RATE/16)-1)
    UBRR0L=((unsigned char)UART_BAUD_SELECT);
    UDR0 = 0xAA;  
#endif // USE_SPEKTRUM

    Sleepy::loseSomeTime(40); // wait for rfm-startup

#ifdef USE_RADIO_RFM12
    rf12_initialize(NODE_SLAVE, RF12_868MHZ, 212); // NodeID, Freq, netGroup
    rf12_initialize(NODE_SLAVE, RF12_868MHZ, 212); // NodeID, Freq, netGroup
    rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
    msg_received = (masterCTRL*) rf12_data;
#endif // USE_RADIO_RFM12

#ifdef USE_RADIO_RFM95
    rf95.init();
    rf95.setFrequency(868.0);            // in MHz
    //rf95.setModemConfig(Bw125Cr45Sf128); // Medium Range (lookup _RF95.h)
    rf95.setTxPower(13);                 // 5 ... 23 dBm
    msg_received = (masterCTRL*) &msg_rfm95;
//rf95.sleep();
#endif // USE_RADIO_RFM95
    
    for (uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++)
    {
        msg_valid.analog[ivar] = STDVALUE;
    }
    msg_valid.counter = 255; // correct errorrate-values

    pinMode(        PIN_LED, OUTPUT);
    digitalWrite(   PIN_LED, HIGH);
}



void loop()
{

    uint16_t errorCounter = 0;
    uint8_t  errorDetect = 0;
    uint8_t  mustCTRL = 0;
    uint8_t  couldCTRL = 0;
    uint8_t  dataValid = 0;

    uint32_t loop_time = millis();
    uint32_t time2ctrl_min = loop_time + CONTROL_INTERVALL_MIN;
    uint32_t time2ctrl_max = loop_time + CONTROL_INTERVALL_MAX;
    
    uint16_t spektrumCH[7];
    spektrumCH[0]       = STDVALUE;
    spektrumCH[1]       = STDVALUE;
    spektrumCH[2]       = STDVALUE;
    spektrumCH[3]       = STDVALUE;
    spektrumCH[4]       = STDVALUE;
    spektrumCH[5]       = STDVALUE;
    spektrumCH[6]       = STDVALUE;
    //spektrumCH[7]       = STDVALUE;
    
loop_start:

#ifdef USE_RADIO_RFM12
    if (rf12_recvDone())      dataValid = rfm_handle();
#endif // USE_RADIO_RFM12

#ifdef USE_RADIO_RFM95
    if (rf95.available())     dataValid = rfm_handle();
#endif // USE_RADIO_RFM95

    loop_time     = millis();
    if (loop_time > time2ctrl_max)     mustCTRL  = 1;
    
    if (dataValid)
    {
        if (loop_time > time2ctrl_min) {
          mustCTRL        = 1;
          dataValid       = 0;
          errorCounter    = 0;
          errorDetect     = 0;
        }
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

        // Normale Belegung (MODE2)
        //              Vertikal        Horizontal
        // Links:   |   Throttle,     - Rudder/YAW
        // Rechts:  |   Elevator/Roll - Aileron/Nick/Pitch

        //  CH0 R H RechtsPlus
        //  CH1 R V ObenMinus
        //  CH2 L V ObenPlus
        //  CH3 L H RechtsMinus
        servo[0]   = (servo[0]-512)*1.40+512;
        servo[1]   = (servo[1]-512)*1.40+512;
        servo[2]   = (servo[2]-512)*1.40+512; //trottle
        servo[3]   = (servo[3]-512)*1.40+512;
        
        if      (servo[5] > 512)   servo[5] -= 20;
        else if (servo[5] < 512)   servo[5] += 20;
                
        spektrumCH[0] = servo[0];
        spektrumCH[1] = 1024 - servo[1];
        spektrumCH[2] = servo[2];
        spektrumCH[3] = 1024 - servo[3];
        spektrumCH[4] = servo[5];

#ifdef USE_SPEKTRUM
        spektrumSerial_send(spektrumCH);
#endif // USE_SPEKTRUM

        time2ctrl_min = loop_time + CONTROL_INTERVALL_MIN;
        time2ctrl_max = loop_time + CONTROL_INTERVALL_MAX;
        mustCTRL = 0;
    }

    goto loop_start;

}




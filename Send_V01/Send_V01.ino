/*

TODO:
- transfer code-blocks to libs
- include all in one?

*/

//#define         USE_RADIO_RFM12
#define         USE_RADIO_RFM95     // TODO: switch to RHReliableDatagram

//#define         USE_SERIAL
#define         SERIAL_TX_BUFFER_SIZE   (16)
#define         SERIAL_RX_BUFFER_SIZE   (4)
#define         SERIAL_BUFFER_SIZE      (16)

const uint8_t   NODE_MASTER             = ( 9 );
const uint8_t   NODE_SLAVE              = ( 10 );

const uint8_t   MESSAGE_INTERVALL_MIN   = ( 60 ); // standard: 20
const uint8_t   MESSAGE_INTERVALL_MAX   = ( 200 );

#define         USE_VCCSTARTCHECK
const uint32_t  VCC_MIN                 = ( 2900 ); // mV

#define         USE_POWEROFF
const uint32_t  POWEROFF_INTERVALL      = ( ( 5 * 60 * 1000L ) ); // ms

const uint8_t   ANALOG_CHANNELS         = ( 6 ); // number of channels to transmit
const uint16_t  ANALOG_JITTER           = ( 2 );
const uint16_t  ANALOG_SAFEZONE	        = ( 100 );

// Config Module AS2015#A
const uint8_t   DIGITAL_CHANNELS        = ( 0 ); // how much are used?
const uint8_t   PIN_DIGITAL_A           = ( 0 );
const uint8_t   PIN_DIGITAL_B           = ( 1 );
const uint8_t   PIN_DIGITAL_C           = ( 3 );
const uint8_t   PIN_DIGITAL_D           = ( 4 );
const uint8_t   PIN_DIGITAL_E           = ( 5 ); // TODO: 4 INPuT; 4 OUTPUT?
const uint8_t   PIN_DIGITAL_F           = ( 6 );
const uint8_t   PIN_DIGITAL_G           = ( 7 );
const uint8_t   PIN_DIGITAL_H           = ( 9 );

#define         USE_LATENCYTEST     // Test-Output which triggers a gpio for timing-analysis: high when can_send, low after ADC-sampling
// rfm95 takes about 60ms from reading ADC, over sending, to writing to naze32
const uint8_t   PIN_LED                 = ( 9 ); // standard: 8
const uint8_t   PIN_LATENCY             = ( 8 ); // INT 1

// Config for Module_Modellbau
#ifdef USE_RADIO_RFM12
const uint8_t   PIN_RX_PWR              = ( 5 );
const uint8_t   PIN_TX_PWR              = ( 6 );
#endif // USE_RADIO_RFM12

/////////////////////  END OF CONFIG  //////////////////////////////////

#include <avr/wdt.h>

/// PowerSaver-Lib
#include <PowerSaver.h>
PowerSaver ps;

ISR ( WDT_vect )
{
        ++vector_wdt_called;
};

#ifdef USE_RADIO_RFM12
#include <JeeLib.h> // JeeLib RF
#include <RF12sio.h>
RF12    RF12;
const uint8_t   SEND_MODE               = ( 2 ); // set to 3 if fuses are e=06/h=DE/l=CE, else set to 2
const uint8_t   HDR_MASTER_MSG          = ( ( ( NODE_SLAVE&RF12_HDR_MASK ) | ( RF12_HDR_DST | 0 ) ) );
const uint8_t   HDR_MASTER_ACK          = ( ( ( NODE_SLAVE&RF12_HDR_MASK ) | ( RF12_HDR_CTL | RF12_HDR_DST ) ) );
const uint8_t   HDR_SLAVE_MSG           = ( ( ( NODE_SLAVE&RF12_HDR_MASK ) | ( 0 ) ) );
const uint8_t   HDR_SLAVE_ACK           = ( ( ( NODE_SLAVE&RF12_HDR_MASK ) | ( RF12_HDR_CTL ) ) );
#endif  // USE_RADIO_RFM12

#ifdef USE_RADIO_RFM95
#include <SPI.h>
#include <RH_RF95.h>
RH_RF95 rf95; // Singleton instance of the radio driver
#undef YIELD
#define YIELD   (ps.sleep ( 0 ) )
#endif // USE_RADIO_RFM95

#ifdef USE_VCCSTARTCHECK
#include <atmel_vcc.h>
ATMEL atmel = ATMEL ();
volatile bool adcDone;
ISR ( ADC_vect )
{
        adcDone = true;
};
#endif // USE_VCCSTARTCHECK

struct masterCTRL
{
        uint8_t  com;
        uint8_t  counter;
        uint8_t  digital;
        uint16_t analog[ANALOG_CHANNELS]; // 8xADC + 1xVCC
        uint16_t vcc;
        //uint32_t time;
};

const uint8_t analog_size  = ANALOG_CHANNELS * sizeof ( uint16_t );
const uint8_t msg_size     = sizeof ( masterCTRL );

struct masterTEMP
{
        uint8_t digital;
        uint16_t analog[ANALOG_CHANNELS]; // previous "long"
};



/////////////////////  PROGRAM:  //////////////////////////////////

void setup ()
{
        wdt_enable ( WDTO_1S );

#ifdef USE_SERIAL
        Serial.begin ( 115200 );
#endif

        ps.turnOffTWI ();
        ps.turnOffTimer1 ();
        ps.turnOffTimer2 ();

        ps.sleep ( 2 ); // wait for rfm-startup

#ifdef USE_RADIO_RFM12
        //rf12_config();
        rf12_initialize ( NODE_MASTER, RF12_868MHZ, 212 ); // NodeID, Freq, netGroup
        rf12_initialize ( NODE_MASTER, RF12_868MHZ, 212 );
        rf12_control ( 0xC040 ); // set low-battery level to 2.2V i.s.o. 3.1V
        rf12_sleep ( RF12_SLEEP );
#endif

#ifdef USE_RADIO_RFM95
        rf95.init ();
        rf95.setFrequency ( 868.0 );          // in MHz
        //rf95.setModemConfig(Bw125Cr45Sf128); // Medium Range (lookup _RF95.h)
#define REG_MODEM_CONFIG1           0x1D        /// register-defines in Radiohead Lib are WRONG!!!
#define     MSK_MODEM_BW            (B11110000)
#define         VAL_MODEM_BW008     (B00000000) // in kHz
#define         VAL_MODEM_BW010     (B00010000)  // in kHz
#define         VAL_MODEM_BW016     (B00100000)  // in kHz
#define         VAL_MODEM_BW021     (B00110000)  // in kHz
#define         VAL_MODEM_BW031     (B01000000)  // in kHz
#define         VAL_MODEM_BW042     (B01010000)  // in kHz
#define         VAL_MODEM_BW063     (B01100000)  // in kHz
#define         VAL_MODEM_BW125     (B01110000)  // in kHz
#define         VAL_MODEM_BW250     (B10000000)  // in kHz
#define         VAL_MODEM_BW500     (B10010000)  // in kHz
#define     MSK_MODEM_CR            (B00001110)
#define         VAL_MODEM_CR1       (B00000010) // Coding Rate 4/5
#define         VAL_MODEM_CR2       (B00000100) // Coding Rate 4/6
#define         VAL_MODEM_CR3       (B00000110) // Coding Rate 4/7
#define         VAL_MODEM_CR4       (B00001000) // Coding Rate 4/8
#define     MSK_MODEM_IMPLICITHDR   (B00000001) // implicit header stores length
#define REG_MODEM_CONFIG2           0x1E
#define     MSK_MODEM_SF            (B11110000)
#define         VAL_MODEM_SF06      (B01100000) // 64 chips/symbol --> SNR -5 dB
#define         VAL_MODEM_SF07      (B01110000) // 128 chips/symbol --> SNR -7.5 dB
#define         VAL_MODEM_SF08      (B10000000) // 256 chips/symbol --> SNR -10 dB
#define         VAL_MODEM_SF09      (B10010000) // 512 chips/symbol --> SNR -12.5 dB
#define         VAL_MODEM_SF10      (B10100000) // 1024 chips/symbol --> SNR -15 dB
#define         VAL_MODEM_SF11      (B10110000) // 2048 chips/symbol --> SNR -17.5 dB
#define         VAL_MODEM_SF12      (B11000000) // 4096 chips/symbol --> SNR -20 dB
#define     MSK_TX_CONTINOUOS       (B00001000)
#define     MSK_RX_PAYLOAD_CRC_ON   (B00000100)
#define     MSK_SYMB_TIMEOUTMSB     (B00000011)
#define REG_MODEM_CONFIG3           0x26
#define     MSK_LOW_DATARATE_OPTI   (B00001000) // Mandatory when symbollength > 16ms
#define     MSK_AGC_AUTO_ON         (B00000100)
        RH_RF95::ModemConfig conf = {0,0,0};
        conf.reg_1d = (VAL_MODEM_BW125 | VAL_MODEM_CR1);
        conf.reg_1e = (VAL_MODEM_SF07  | MSK_RX_PAYLOAD_CRC_ON );
        conf.reg_26 = (MSK_AGC_AUTO_ON ); // TODO: Test
        rf95.setModemRegisters(&conf);

        rf95.setTxPower ( 20 );               // 5 ... 23 dBm
        rf95.setPreambleLength ( 8 );
        rf95.sleep ();
#endif // USE_RADIO_RFM95

        // init Pins
#if DIGITAL_CHANNELS
        pinMode (    PIN_DIGITAL_A,  INPUT );
        pinMode (    PIN_DIGITAL_B,  INPUT );
        pinMode (    PIN_DIGITAL_C,  INPUT );
        pinMode (    PIN_DIGITAL_D,  INPUT );

        pinMode (    PIN_DIGITAL_E,  INPUT );
        pinMode (    PIN_DIGITAL_F,  INPUT );
        pinMode (    PIN_DIGITAL_G,  INPUT );
        pinMode (    PIN_DIGITAL_H,  INPUT );
#endif // DIGITAL_CHANNELS


        // Mod for RFM12BP
#ifdef USE_RADIO_RFM12
        pinMode (       PIN_RX_PWR, OUTPUT );
        digitalWrite (  PIN_RX_PWR, LOW );

        pinMode (       PIN_TX_PWR, OUTPUT );
        digitalWrite (  PIN_TX_PWR, LOW );
#endif // USE_RADIO_RFM12

        pinMode (       PIN_LED,    OUTPUT );
        digitalWrite (  PIN_LED,    LOW );

#ifdef USE_LATENCYTEST
        pinMode (       PIN_LATENCY, OUTPUT );
        digitalWrite (  PIN_LATENCY, LOW );
#endif // USE_LATENCYTEST

#ifdef USE_VCCSTARTCHECK
        // Safe-VCC-Check
        if ( atmel.readVcc () < VCC_MIN )
        {
                while ( 1 )     ps.sleep ( 6 );
        }
#endif // USE_VCCSTARTCHECK
}


void loop ()
{

        // timing
        uint32_t loop_time      = millis ();
        uint32_t time2send_min  = loop_time + MESSAGE_INTERVALL_MIN;
        uint32_t time2send_max  = loop_time + MESSAGE_INTERVALL_MAX;
        uint32_t time2powerOff  = loop_time + POWEROFF_INTERVALL;

        // init vars
        struct masterCTRL msg_send;
        struct masterTEMP msg_old;

        msg_send.counter = 0;
        msg_send.com     = analog_size;

        for ( uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++ )
        {
                msg_send.analog[ivar] = analogRead ( ivar ) + ANALOG_SAFEZONE;
                msg_old.analog[ivar]  = msg_send.analog[ivar];
        }

        // state-machine-Vars
        static uint8_t couldSend, newData, mustSend, shouldSend, led_on;

loop_start:

        wdt_reset();

        // delay to while
        delay ( 1 );

        // READ DIGITAL
#if DIGITAL_CHANNELS
        if ( digitalRead ( PIN_DIGITAL_A ) )    msg_send.digital |=  ( 1 < 0 );
        else                                    msg_send.digital &= ~ ( 1 < 0 );
        if ( digitalRead ( PIN_DIGITAL_B ) )    msg_send.digital |=  ( 1 < 1 );
        else                                    msg_send.digital &= ~ ( 1 < 1 );
        if ( digitalRead ( PIN_DIGITAL_C ) )    msg_send.digital |=  ( 1 < 2 );
        else                                    msg_send.digital &= ~ ( 1 < 2 );
        if ( digitalRead ( PIN_DIGITAL_D ) )    msg_send.digital |=  ( 1 < 3 );
        else                                    msg_send.digital &= ~ ( 1 < 3 );
        if ( digitalRead ( PIN_DIGITAL_E ) )    msg_send.digital |=  ( 1 < 4 );
        else                                    msg_send.digital &= ~ ( 1 < 4 );
        if ( digitalRead ( PIN_DIGITAL_F ) )    msg_send.digital |=  ( 1 < 5 );
        else                                    msg_send.digital &= ~ ( 1 < 5 );
        if ( digitalRead ( PIN_DIGITAL_G ) )    msg_send.digital |=  ( 1 < 6 );
        else                                    msg_send.digital &= ~ ( 1 < 6 );
        if ( digitalRead ( PIN_DIGITAL_H ) )    msg_send.digital |=  ( 1 < 7 );
        else                                    msg_send.digital &= ~ ( 1 < 7 );
#endif // DIGITAL_CHANNELS


        loop_time = millis ();
        if ( loop_time >= time2send_max ) mustSend  = 1;
        if ( loop_time >= time2send_min ) couldSend = 1;
        else goto loop_start;

#ifdef USE_LATENCYTEST
        digitalWrite ( PIN_LATENCY, HIGH );
#endif // USE_LATENCYTEST

        // READ ANALOG
        for ( uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++ )
        {
                msg_send.analog[ivar] = analogRead ( ivar ) + ANALOG_SAFEZONE; // ADD safezone to avoid checking...
                if      ( msg_send.analog[ivar] >= msg_old.analog[ivar] + ANALOG_JITTER )  newData = 1;
                else if ( msg_send.analog[ivar] <= msg_old.analog[ivar] - ANALOG_JITTER )  newData = 1;
        }

#ifdef USE_RADIO_RFM12
        // Mod for RFM12BP
        if ( msg_send.analog[4] > 512 + ANALOG_SAFEZONE )
        {
                digitalWrite ( PIN_TX_PWR, HIGH );
                digitalWrite ( PIN_RX_PWR, LOW );
                led_on = 1;
        }
        else
        {
                digitalWrite ( PIN_TX_PWR, LOW );
                digitalWrite ( PIN_RX_PWR, LOW ); // Fragment of Receive-Debug
                led_on = 0;
        }
#endif // USE_RADIO_RFM12

        if ( couldSend )  // FLOW: couldSend stays 1 till newData gets in, immediately send out!
        {
                if ( newData )
                {
                        shouldSend    = 5; // retry rate
                        newData       = 0;
                        time2powerOff = loop_time + POWEROFF_INTERVALL;
                }
                if ( shouldSend )
                {
                        mustSend      = 1;
                        couldSend     = 0;
                        shouldSend--;
                }
        }

        if ( mustSend )
        {

#ifdef USE_LATENCYTEST
                digitalWrite ( PIN_LATENCY, LOW );
#endif // USE_LATENCYTEST

#ifdef USE_RADIO_RFM12
                static boolean received;
                rf12_sleep ( RF12_WAKEUP );
                while ( !rf12_canSend() )
                {
                        received |= rf12_recvDone();
                }
                rf12_sendNow ( HDR_MASTER_MSG, &msg_send, msg_size );
                rf12_sendWait ( SEND_MODE );
                rf12_sleep ( RF12_SLEEP );
#endif

#ifdef USE_RADIO_RFM95
                rf95.send ( ( uint8_t * ) & msg_send, msg_size );
                rf95.waitPacketSent ();
                rf95.sleep ();
#endif // USE_RADIO_RFM95

                if ( msg_send.counter & led_on ) digitalWrite ( PIN_LED, HIGH );
                else                             digitalWrite ( PIN_LED, LOW );

                msg_send.counter++;

                // Backup old values
                //strncpy((char*) &msg_old.analog[0],(char*) &msg_send.analog[0], analog_size);
                for ( uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++ )
                {
                        msg_old.analog[ivar] = msg_send.analog[ivar];
                }

                time2send_min = loop_time + MESSAGE_INTERVALL_MIN;
                time2send_max = loop_time + MESSAGE_INTERVALL_MAX;

                mustSend  = 0;
                couldSend = 0;

#ifdef USE_VCCSTARTCHECK
                // read vcc and store it for next time
                msg_send.vcc = atmel.readVcc ();
#endif // USE_VCCSTARTCHECK

#ifdef USE_SERIAL
                Serial.print ( msg_send.counter );
                Serial.print ( " : " );
                for ( byte ivar = 0; ivar < ANALOG_CHANNELS; ivar++ )
                {
                        Serial.print ( msg_send.analog[ivar] );
                        Serial.print ( " : " );
                }
                Serial.println ( msg_send.vcc );
                Serial.print ( " : " );
                Serial.println ( msg_send.digital, BIN );
                Serial.println ( "" );
#endif
        }
        //else    ps.sleep ( 0 );

#ifdef USE_POWEROFF
        if ( loop_time >= time2powerOff )
        {
#ifdef USE_USE_RADIO_RFM12
                digitalWrite ( PIN_TX_PWR, LOW );
                digitalWrite ( PIN_RX_PWR, LOW ); // Fragment of Receive-Debug
                rf12_sleep ( RF12_SLEEP );
#endif // USE_USE_RADIO_RFM12
                digitalWrite ( PIN_LED,     LOW );
                ps.sleep ( 6 );
        }
#endif // USE_POWEROFF

        goto loop_start;
}

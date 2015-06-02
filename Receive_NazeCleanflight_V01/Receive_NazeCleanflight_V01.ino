//#define USE_RADIO_RFM12
#define USE_RADIO_RFM95
#define USE_SPEKTRUM    // http://blog.kwarf.com/2013/08/a-look-at-the-spektrum-satellite-protocol/

const uint8_t   NODE_MASTER             = ( 9 );
const uint8_t   NODE_SLAVE              = ( 10 );

const uint8_t   ANALOG_CHANNELS         = ( 6 );	// nr of channels
const uint16_t  ANALOG_JITTER           = ( 2 );	// small changes won't trigger fast sending
const uint16_t  ANALOG_SAFEZONE         = ( 100 );	// better use of uint16 (move away from 0)
const uint16_t  ANALOG_DEADZONE         = ( 8 );	// for ignoring small drifts (around zero)
const uint16_t  STDVALUE                = ( 512 );	// middle of [0...1023]

#define         USE_LATENCYTEST
const uint8_t   PIN_LED                 = ( 9 );        // standard: 8
const uint8_t   PIN_LATENCY             = ( 8 ); 	// INT 1

const uint32_t  CONTROL_INTERVALL_MIN   = ( 7L );
const uint32_t  CONTROL_INTERVALL_MAX   = ( 18L );
// cleanflight is ok with 5ms distance! https://github.com/cleanflight/cleanflight/blob/master/src/main/rx/spektrum.c
// 16byte Frame takes about 1.5ms over serial

const uint16_t  ERROR_TIME_MS           = ( 5000L );
const uint16_t  ERROR_THRESHOLD         = ( ceil ( ERROR_TIME_MS / CONTROL_INTERVALL_MAX ) );
const uint16_t  ERROR_SCALEFAK          = ( 80L ); // to get slower fallrate / has to be lower than threshold
const uint16_t  ERROR_FALLRATE          = ( ceil ( 200.0 * ERROR_SCALEFAK * CONTROL_INTERVALL_MAX / 30000.0 ) ); // lower Throttle 200n in 30sec
const uint16_t  ERROR_THROTTLE_MIN      = ( 350 );	// drop till this point

/////////////////////  END OF CONFIG  //////////////////////////////////

#include <avr/wdt.h>

#define         SERIAL_TX_BUFFER_SIZE   (16)
#define         SERIAL_RX_BUFFER_SIZE   (4)
#define         SERIAL_BUFFER_SIZE      (16)

#ifdef USE_RADIO_RFM12
#include <JeeLib.h>
#include <RF12sio.h>
RF12 RF12;
const uint8_t   SEND_MODE               = ( 2 ); // set to 3 if fuses are e=06/h=DE/l=CE, else set to 2
const uint8_t   HDR_MASTER_MSG          = ( ( ( NODE_SLAVE&RF12_HDR_MASK ) | ( RF12_HDR_DST | 0 ) ) ); // |RF12_HDR_ACK if you want an ACK
const uint8_t   HDR_MASTER_ACK          = ( ( ( NODE_SLAVE&RF12_HDR_MASK ) | ( RF12_HDR_CTL | RF12_HDR_DST ) ) );
const uint8_t   HDR_SLAVE_MSG           = ( ( ( NODE_SLAVE&RF12_HDR_MASK ) | ( 0 ) ) );
const uint8_t   HDR_SLAVE_ACK           = ( ( ( NODE_SLAVE&RF12_HDR_MASK ) | ( RF12_HDR_CTL ) ) );
#endif // USE_RADIO_RFM12


/// PowerSaver-Lib
#include <PowerSaver.h>
PowerSaver ps;
ISR ( WDT_vect )
{
        ++vector_wdt_called;
};


#ifdef USE_RADIO_RFM95
#include <SPI.h>
#include <RH_RF95.h>
RH_RF95 rf95; // Singleton instance of the radio driver
#undef YIELD
#define YIELD   (ps.sleep(0))
#endif // USE_RADIO_RFM95


#include <atmel_vcc.h>
ATMEL atmel = ATMEL ();
volatile bool adcDone;
ISR ( ADC_vect )
{
        adcDone = true;
};

#ifdef USE_SPEKTRUM
#include <atmel_spektrumSerial.h>
#endif // USE_SPEKTRUM


struct masterCTRL
{
        uint8_t com;
        uint8_t counter;
        uint8_t digital;
        uint16_t analog[ANALOG_CHANNELS]; // 8xADC + 1xVCC
        uint16_t vcc;
        //uint32_t    time;
};
masterCTRL *msg_received; /// HIER IST EINE ÄNDERUNG in Bezug zur Sendeversion
masterCTRL msg_valid, msg_rfm95;
const uint8_t msg_size = sizeof ( masterCTRL );

uint16_t receive_errors; // use for QoS

/////////////////////  PROGRAM:  //////////////////////////////////

uint8_t rfm_handle ()
{

#ifdef USE_RADIO_RFM12
        if ( rf12_crc != 0 )                                    return 0;
        if ( ( rf12_hdr & RF12_HDR_MASK ) != NODE_SLAVE )       return 0;
        if ( rf12_len != msg_size )                             return 0;
#endif // USE_RADIO_RFM12

#ifdef USE_RADIO_RFM95
        uint8_t rf95_len = RH_RF95_MAX_MESSAGE_LEN;
        if ( !rf95.recv ( ( uint8_t* ) &msg_rfm95, &rf95_len ) )   return 0;
        if ( rf95_len != msg_size )                                return 0;
        // TODO check CRC
#endif // USE_RADIO_RFM95

#ifdef USE_LATENCYTEST
        digitalWrite (  PIN_LATENCY, HIGH );
#endif // USE_LATENCYTEST

        int16_t puffer[ANALOG_CHANNELS];

        for ( uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++ )
        {
                puffer[ivar] = msg_received->analog[ivar];
                if ( puffer[ivar] > 1024 + ANALOG_SAFEZONE )    return 0;
                if ( puffer[ivar] < ANALOG_SAFEZONE )           return 0;
        }

        if ( msg_received->counter == uint8_t ( msg_valid.counter ) ) 		return 0;     // already received it
        if ( msg_received->counter != uint8_t ( msg_valid.counter + 1 ) ) 	receive_errors++; // for statistics / later

        // strncpy((char*) &msg_valid,(char*) msg_received, msg_size); // doenst work as expected / skips bytes?
        msg_valid.com 		= msg_received->com;
        msg_valid.counter 	= msg_received->counter;
        msg_valid.digital 	= msg_received->digital;
        msg_valid.vcc 		= msg_received->vcc;

        for ( uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++ )
        {
                msg_valid.analog[ivar] = puffer[ivar] - ANALOG_SAFEZONE;
        }

        if ( msg_received->counter & 1 )        digitalWrite ( PIN_LED, HIGH );
        else                                    digitalWrite ( PIN_LED, LOW );

        //rssi = rf12_control();
        return 1;
}


void setup ()
{
        wdt_enable (WDTO_1S);

        ps.turnOffTWI ();
        ps.turnOffTimer1 ();
        ps.turnOffTimer2 ();
        ps.turnOffDigitalInput ();

        ps.sleep ( 1 ); // wait for rfm-startup

#ifdef USE_RADIO_RFM12
        rf12_initialize ( NODE_SLAVE, RF12_868MHZ, 212 ); // NodeID, Freq, netGroup
        rf12_initialize ( NODE_SLAVE, RF12_868MHZ, 212 ); // NodeID, Freq, netGroup
        rf12_control ( 0xC040 ); // set low-battery level to 2.2V i.s.o. 3.1V
        msg_received = ( masterCTRL * ) rf12_data;
#endif // USE_RADIO_RFM12

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
        msg_received = ( masterCTRL* ) &msg_rfm95;
#endif // USE_RADIO_RFM95

        for ( uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++ )
        {
                msg_valid.analog[ivar] = STDVALUE;
        }
        msg_valid.counter = 255; // correct errorrate-values

        pinMode (       PIN_LED, OUTPUT );
        digitalWrite (  PIN_LED, HIGH );

#ifdef USE_LATENCYTEST
        pinMode (       PIN_LATENCY, OUTPUT );
        digitalWrite (  PIN_LATENCY, LOW );
#endif // USE_LATENCYTEST
}


void loop ()
{

#ifdef USE_SPEKTRUM
        spektrum_init ();
        //#define UART_BAUD_RATE 104200   // hacker bei ROLL
#define UART_BAUD_RATE 107500   // hacker bei ROLL
#define UART_BAUD_SELECT ((F_CPU/UART_BAUD_RATE/16)-1)
        UBRR0L = ( ( unsigned char ) UART_BAUD_SELECT );
        UDR0 = 0xAA;
#endif // USE_SPEKTRUM

        uint16_t  errorCounter  = 0;
        uint8_t   errorDetect   = 0;
        uint8_t   mustCTRL      = 0;
        uint8_t   couldCTRL     = 0;
        uint8_t   dataValid     = 0;

        uint32_t  loop_time     = millis ();
        uint32_t  time2ctrl_min = loop_time + CONTROL_INTERVALL_MIN;
        uint32_t  time2ctrl_max = loop_time + CONTROL_INTERVALL_MAX;

        uint16_t  spektrumCH[12]= {STDVALUE, STDVALUE, STDVALUE, STDVALUE, STDVALUE, STDVALUE, STDVALUE, STDVALUE, STDVALUE, STDVALUE, STDVALUE, STDVALUE};

loop_start:

        wdt_reset ();

#ifdef USE_RADIO_RFM12
        if ( rf12_recvDone () ) 	dataValid = rfm_handle ();
#endif // USE_RADIO_RFM12

#ifdef USE_RADIO_RFM95
        if ( rf95.available() )	dataValid = rfm_handle();
#endif // USE_RADIO_RFM95

        loop_time = millis ();
        if ( loop_time > time2ctrl_max ) mustCTRL = 1;

        if ( dataValid )
        {
                if ( loop_time > time2ctrl_min )
                {
                        mustCTRL = 1;
                        dataValid = 0;
                        errorCounter = 0;
                        errorDetect = 0;
                }
        }

        if ( mustCTRL )
        {
                if ( errorCounter > ERROR_THRESHOLD )
                {

                        errorCounter -= ERROR_SCALEFAK;

                        // place safe value
                        msg_valid.analog[0] = STDVALUE;
                        msg_valid.analog[1] = STDVALUE;

                        if ( errorDetect == 0 )
                        {
                                errorDetect = 1;
                                msg_valid.analog[2] = STDVALUE + 50;
                        }

                        if ( msg_valid.analog[2] > ERROR_THROTTLE_MIN ) 	msg_valid.analog[2] -= ERROR_FALLRATE;
                        else 											msg_valid.analog[2] = ERROR_THROTTLE_MIN;

                        msg_valid.analog[3] = STDVALUE;
                        msg_valid.analog[4] = STDVALUE;
                        msg_valid.analog[5] = 1023; // switch to stable Mode!

                        msg_valid.counter = 255; // correct errorrate-values
                }
                else errorCounter++; // will reset if valid paket comes in

                int16_t servo[ANALOG_CHANNELS];

                for ( uint8_t ivar = 0; ivar < ANALOG_CHANNELS; ivar++ )
                {
                        if 	( msg_valid.analog[ivar] > STDVALUE + ANALOG_DEADZONE )	servo[ivar] = msg_valid.analog[ivar] - ANALOG_DEADZONE;
                        else if ( msg_valid.analog[ivar] < STDVALUE - ANALOG_DEADZONE )	servo[ivar] = msg_valid.analog[ivar] + ANALOG_DEADZONE;
                        else 								servo[ivar] = STDVALUE;
                }

                // Normale Belegung (MODE2)
                //              Vertikal        Horizontal
                // Links:   |   Throttle,     - Rudder/YAW
                // Rechts:  |   Elevator/Roll - Aileron/Nick/Pitch

                //  CH0 R H RechtsPlus
                //  CH1 R V ObenMinus
                //  CH2 L V ObenPlus
                //  CH3 L H RechtsMinus
                servo[0] = ( servo[0] - 512 ) * 1.40 + 512;
                servo[1] = ( servo[1] - 512 ) * 1.40 + 512;
                servo[2] = ( servo[2] - 512 ) * 1.40 + 512; //trottle
                servo[3] = ( servo[3] - 512 ) * 1.40 + 512;

                if ( servo[5] > 512 ) servo[5] -= 20;
                else if ( servo[5] < 512 ) servo[5] += 20;

                spektrumCH[0] = servo[0];
                spektrumCH[1] = 1024 - servo[1];
                spektrumCH[2] = servo[2];
                spektrumCH[3] = 1024 - servo[3];
                spektrumCH[4] = servo[5];

#ifdef USE_SPEKTRUM
                spektrum_send ( spektrumCH );
#endif // USE_SPEKTRUM

#ifdef USE_LATENCYTEST
        digitalWrite (  PIN_LATENCY, LOW );
#endif // USE_LATENCYTEST

                time2ctrl_min = loop_time + CONTROL_INTERVALL_MIN;
                time2ctrl_max = loop_time + CONTROL_INTERVALL_MAX;
                mustCTRL = 0;
        }

        goto loop_start;

}




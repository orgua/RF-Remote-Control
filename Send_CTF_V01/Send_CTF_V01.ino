/*  DOC:

*/

#define USE_RADIO
//#define USE_SERIAL
#define USE_LED

const uint8_t PIN_LED         = 8;
const uint8_t NODE_MASTER			= (11);
const uint8_t NODE_SLAVE      = (NODE_MASTER+1);

/////////////////////  END OF CONFIG  //////////////////////////////////

#include <JeeLib.h> // JeeLib RF

static volatile uint8_t watchdogCounter;
ISR(WDT_vect)
		{
				++watchdogCounter;
		};   // this must be defined since we're using the watchdog for low-power waiting

#ifdef USE_RADIO

#include <RF12sio.h>

RF12 RF12;
#define SEND_MODE               2   // set to 3 if fuses are e=06/h=DE/l=CE, else set to 2
#define HDR_MASTER_MSG          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_DST|0))
#define HDR_MASTER_ACK          ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL|RF12_HDR_DST))
#define HDR_SLAVE_MSG           ((NODE_SLAVE&RF12_HDR_MASK)|(0))
#define HDR_SLAVE_ACK           ((NODE_SLAVE&RF12_HDR_MASK)|(RF12_HDR_CTL))
#endif  // USE_RADIO

char msg = "ds13ctf4e0f67abc";

uint8_t msg_size = sizeof (msg);


/////////////////////  PROGRAM:  //////////////////////////////////


void setup ()
{
#ifndef USE_LED
  pinMode (PIN_LED, OUTPUT);    // help to see if node started up right (short blink)
  digitalWrite (PIN_LED, HIGH);
#endif

#ifdef USE_RADIO
	Sleepy::loseSomeTime (40);
	rf12_initialize (NODE_MASTER, RF12_868MHZ, 212); // NodeID, Freq, netGroup
	rf12_initialize (NODE_MASTER, RF12_868MHZ, 212);
	rf12_control (0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
	rf12_sleep (RF12_SLEEP);
#endif

#ifdef USE_LED
  digitalWrite (PIN_LED, LOW);
#else 
	digitalWrite (PIN_LED, LOW);
	pinMode (PIN_LED, INPUT);
#endif // USE_LED
}

void loop ()
{
  uint16_t sleep_intervall = 1000;
	uint32_t next_time = millis() + sleep_intervall;

	loop_start:

	if (millis () <= next_time) goto loop_start;

#ifdef USE_LED
     digitalWrite(PIN_LED, HIGH);
#endif // USE_LED

  next_time += sleep_intervall;
  
  
#ifdef USE_RADIO
		static boolean received;
		uint8_t send_try = 250;

		rf12_sleep (RF12_WAKEUP);
		while ((!rf12_canSend ()) && send_try)
		{
			received |= rf12_recvDone ();
			send_try--;
		}
   
		if (send_try)
		{
			rf12_sendNow (HDR_MASTER_MSG, &msg, msg_size);
			rf12_sendWait (SEND_MODE);
			rf12_sleep (RF12_SLEEP);
		}
#endif

#ifdef USE_LED
     digitalWrite(PIN_LED, LOW);
#endif // USE_LED

	goto loop_start;
}

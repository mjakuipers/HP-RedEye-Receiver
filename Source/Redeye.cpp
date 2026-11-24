/*
 ************************************************************************
 * A decoder for the HP82240 infrared printer protocol.
 ************************************************************************
 *
 * Copyright (C) 2014-2015 Martin Hepperle.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * Version modified by Meindert Kuipers to support the Arduino Pro Micro
 * adds support for controlling the LED on the C4103A housing with pin D5
 * Uses pin D3 for the IRQ output for testing
 * 
 ************************************************************************
 * 
 * This sketch was written for an Arduino Leonardo with the IR sensor
 * attached to digital pin #2 or digital pin #6.
 *
 * I wanted to avoid the indirection via a jump table by the Arduino interrupt
 * functions attachInterrupt() and detach Interrupt(). Therefore the low level 
 * interrupt service routines ISR() used.
 * If you want to use a different board, you have to adapt several things.
 *
 * ========
 * Leonardo
 * ========
 *                        +----------+
 *                        |          |
 *                        |          |
 *                        |     A    |
 *                        |     r    | D7 <---- IR RX (if INT6 is used)
 *  IR receiver <--- 5.0V |     d    | D6
 *                        |     u    | D5
 *                        |     i    | D4 <---- mode switch --[ 1k ]---| GND
 *                        |     n    | D3 <---- optional debug output -> OSCI
 *  IR receiver <---- GND |     o    | D2 <---- IR RX (if INT1 is used)
 *                        |          | D1 
 *                        |          | D0
 *                        +----------+
 *
 * ========
 * Pro Mini
 * ========
 *                        +----------+
 *                        |          | VCC, GND, TX, RX from USB-TTL cable
 *                        |          |
 *                        |     A    |
 *                        |     r    | D7
 *  IR receiver <--- 5.0V |     d    | D6
 *                        |     u    | D5
 *                        |     i    | D4 ----> TX IR-LED (not used)
 *                        |     n    | D3 <---- IR RX (if INT1 is used) 
 *  IR receiver <---- GND |     o    | D2
 *                        |          | D1
 *                        |          | D0
 *                        +----------+
 *                        
 * ====================                       
 * Pro Micro (Sparkfun)                       
 * ====================       USB
 *                        +---||||---+
 *                        |          |
 *                        |          |
 *  IR receiver <---- GND |    P     |
 *                        |    r  Vcc| +5V ---> IR receiver
 *  IR receiver ----> RxD |D2  o     |
 *        (optional test) |D3        |
 *                        |    M     |
 *  IR receiver <---- LED |D5  i     |
 *                        |    c     |
 *                        |    r     |
 *                        |    o     |
 *                        |          |
 *                        |          |
 *                        +----------+
 *                                              
 *                        
 */
 
/*****************************************************************************/

#include "RedEye.h"


// we can use either INT1 or INT6 (see redeye.h)
// on the Pro Micro, only INT1 should be used!
#if INT_NO == INT1
  // INT1 on PD1 is part of the input pins port D (PIND)
  #define sensor_PORT   PIND
  #ifdef LEONARDO
     // the output of the IR receiver goes to PD1 (physical pin D2 on Leonardo)
     #define sensorPin     1
  #endif
  #ifdef PRO_MINI
     // the output of the IR receiver goes to PD3 (physical pin D3 on Pro Mini
     #define sensorPin     3
  #endif
  
  #ifdef PRO_MICRO
     // the output of the IR receiver goes to PD2 (physical pin D2 on Pro Micro
     #define sensorpin     2  
  #endif
#endif

// note #elsif did not work as desired, therefore multiple #if
#if INT_NO == INT6
  // INT6 on PE6 is part of the input pins port E (PINE)
  #define sensor_PORT   PINE
  // the output of the IR receiver goes to PE6 (physical pin D7 on Leonardo)
  #define sensorPin     6
#endif

/*****************************************************************************/
/* We use a ring buffer filled by the interrupt service routine of timer 0
 * and emptied by the main loop, which ships out the bytes through the
 * serial interface.
 * A value of 1024 defines a large buffer, for simple printing applications
 * from a HP pocket calculator with DELAY of 1.x seconds a value of 8 bytes
 * is sufficient
*/
#define RING  1024
volatile int idxWrite = 0;  // write pointer, accessed from interrupt
volatile int idxRead = 0;   // read pointer, accessed from main loop
static byte databuffer[RING];
/*****************************************************************************/
// some local functions
static void appendToOutput ( const char * pErrorMsg );

/*****************************************************************************
 * if PIN_OUTPUT is defined, the Arduino pin 4 resp. 3 can be used to indicate
 * events so that they can be monitored on an oscilloscope.
 * 
 * for Pro Micro, defince the LED on the IR housing
 * 
 */
  
#ifdef PRO_MINI
  #undef PIN_OUTPUT
  //#define PIN_OUTPUT
  #ifdef PIN_OUTPUT
    // Leonardo: D3 == PD0
    #define signalArdiunoPin  4  // Arduino "pin 4" ...
    #define signalPin         0  // ... is "PD0" on PORTD of the AVR chip
                               // pin used for debug output, e.g. timing crosscheck
    #define signalPORT   PORTD
  #endif
#endif

#ifdef LEONARDO
  #define PIN_OUTPUT
  #ifdef PIN_OUTPUT
    // Leonardo: D3 == PD0
    #define signalArdiunoPin  3  // Arduino "pin 3" ...
    #define signalPin         0  // ... is "PD0" on PORTD of the AVR chip
                               // pin used for debug output, e.g. timing crosscheck
    #define signalPORT   PORTD
  #endif
#endif

#ifdef PRO_MICRO
  #undef PIN_OUTPUT
  #ifdef PIN_TEST
    // Pro Micro: additional test output
    #define TestPin       3  // Pin D3
  #endif
#endif
    

/*****************************************************************************
 * input port D is used for mode selection switch (interpreter/analyzer)
 * If the mode switch is open, the pullup pulls the modePin high. 
 * Then the interpreter mode is ON.
 */
#define MODE_SWITCH
// Leonardo: D4 == PD4
#define modePORT   PIND
// the switch may connect PD4 (physical "pin 4" on Leonardo) to ground
#define modePin     4

// the mode switch is not used on the Pro Micro since it is complete built into the IR Receiver
#ifdef PRO_MINI
  #undef MODE_SWITCH
#endif

/*****************************************************************************
 * Per HP documentation we need 6 to 8 pulses for a valid burst.
 * Note that the HP 48G may send out les than 6 pulses when the
 * batteries are low. Therefore the lower pulse count is relaxed
 * to 5 here, but the solution is to use fresh batteries.
 * The problem manifests itself in losing characters.
 */ 
#define MIN_PULSES 5


/*****************************************************************************
 * MODE_INTERPRETER  all bytes are filtered and passed to the serial interface
 * MODE_ANALYZER     pulse count and gap duration will be output 
 *                   (IR signal analyzer for debugging purposes)
 *
 * Example output in analyzer mode:
 * Sending the string ABC from a HP 48 starts with the escape sequence to 
 * select the character set followed the string enclosed in single quotes.
 * (8)27(8)27(8)53(8)27(8)80(8)27(8)80(8)53(8)54(8)26(8)54(8)80(8)27(8)53(8)
 *       D = '.' = 0x1b, 1/2-bit = 27
 * (8)27(8)27(8)53(8)27(8)80(8)27(8)53(8)54(8)53(8)53(8)54(8)80(8)53(8)27(8)
 *       D = '.' = 0xf9, 1/2-bit = 27
 * (8)26(8)27(8)27(8)53(8)80(8)54(8)53(8)53(8)27(8)80(8)54(8)26(8)54(8)53(8)
 *       D = ''' = 0x27, 1/2-bit = 26
 * (8)27(8)27(8)26(8)54(8)80(8)27(8)80(8)26(8)80(8)53(8)53(8)54(8)53(8)27(8)
 *       D = 'A' = 0x41, 1/2-bit = 27
 * (8)27(8)27(8)26(8)80(8)27(8)53(8)81(8)26(8)80(8)54(8)53(8)54(8)26(8)80(8)
 *       D = 'B' = 0x42, 1/2-bit = 27
 * (8)26(8)27(8)27(8)80(8)53(8)54(8)53(8)27(8)80(8)53(8)54(8)53(8)27(8)53(8)
 *       D = 'C' = 0x43, 1/2-bit = 26
 * (8)27(8)27(8)27(8)53(8)80(8)53(8)54(8)53(8)27(8)80(8)53(8)27(8)54(8)53(8)
 *       D = ''' = 0x27, 1/2-bit = 27
 * (8)27(8)27(8)53(8)27(8)53(8)80(8)54(8)53(8)54(8)53(8)53(8)27(8)80(8)54(8)
 *       D = '.' = 0x04, 1/2-bit = 27
 *
 * The numbers in parentheses are the pulse count of a burst.
 * The numbers between the pulses are the times between the start of
 * subsequent bursts.
 * The D = part shows the character (if printable) and its decimal code.
 * The line ends with the half bit time derived from the start bits.
 */




// Select one mode. I used a variable so that it could also be switched
// at run time
byte mode = MODE_PASSTHROUGH;

// pulse count is restarted for each burst
volatile byte pulse = 0;
// bit count, is restarted for each data frame
volatile int idx = 0;
// the number of timer ticks measured between the three start half-bits
byte halfBitTicks = 25;  // default, will be determined at run time
// the time interval betwen the starting edge of subsequent bursts
byte dt;
/*****************************************************************************/
/*
 * Return true if interpreter mode is ON.
 * This is the case when the mode switch is open and the pullup pulls
 * the pin HIGH.
 */
boolean isInterpreter()
{
  return mode == MODE_INTERPRETER;  
}
/*****************************************************************************/
void setMode ( int newMode )
{
  mode = newMode;
}

int getMode ()
{
  return mode;
}
/*****************************************************************************/
/*
 * Return true if there is at least one new byte in the receiving buffer. 
 *
 * You can call getByte() afterwards to extract the next byte.
 */
boolean isDataAvailable()
{
  return idxWrite != idxRead;
}
/*****************************************************************************/
/*
 * Return the next byte from the receiving buffer. 
 *
 * You should call dataAvailable() first to make sire that there is something
 * in the buffer.
 */
byte getByte()
{
  // there is something in the mailbox      
  // avoid that reading and updating idxRead is interrupted
  // pull the byte from the buffer ...
  byte data = databuffer[idxRead];
  // ... and advance read pointer      
  idxRead = (idxRead+1) % RING;
  
  return data;
}

/*****************************************************************************/
/*
 * This function is called from the usual Arduino setup() function.
 */
void setupRedEye()
{
  //
#ifdef PIN_OUTPUT
  pinMode(signalArdiunoPin, OUTPUT);
  bitClear(PORTD, signalArdiunoPin);
#endif

#ifdef MODE_SWITCH
  // prepare for external mode switch
  pinMode(modePin, INPUT);  
  digitalWrite(modePin,HIGH); // turn on internal pull-up on the modePin
#endif

#ifdef TestPin
  pinMode(TestPin, OUTPUT);
  digitalWrite(TestPin, LOW);
#endif
    
  /************************************************************
   *  we use timer/counter 0 for measuring the time between bursts
   */
  
  // there is no need for action on any pin by timer
  TCCR0A = 0;
  
  // we tick timer 0 at F_CPU/256 = 16/256 MHz = 62.5 kHz
  TCCR0B = (1<<CS02);
  
  // disable the compare output A interrupt
  TIMSK0 = 0;
  
  // set timer output compare A register so that the timer 
  // fires after a maximum burst time of 8 cycles
  OCR0A = (byte)(F_CPU*8L/32768L/256L);

  /************************************************************
   *  We use a interrupt INT_NO to capture the first (FALLING)
   *  edge of each pulse. INT_NO is associated with a certain
   *  Arduino pin.
   */
  // diable interrupts first to avoid interrupt on change
  EIMSK = 0;
  
  // interrupt INT_NO shall occur on the falling edge of the signal
#if INT_NO == INT1
  EICRA = (EICRA & ~((1<<ISC11) | (1<<ISC10))) | (FALLING<<ISC10);
#endif
#if INT_NO == INT6
  EICRB = (EICRB & ~((1<<ISC61) | (1<<ISC60))) | (FALLING<<ISC60);
#endif
  // finally enable the interrupt
  EIMSK |= (1 << INT_NO);
}
/*****************************************************************************/
/*
 * Calculate the error correction nibble for the given data byte.
 * (as per HP 82240B Infrared Printer Technical Interfacing guide))
 */
static byte calcECBits ( byte data )
{
  byte ret = 0x00;
  if ( (data &   1) != 0 ) ret ^= 0x03;
  if ( (data &   2) != 0 ) ret ^= 0x05;
  if ( (data &   4) != 0 ) ret ^= 0x06;
  if ( (data &   8) != 0 ) ret ^= 0x09;
  if ( (data &  16) != 0 ) ret ^= 0x0A;
  if ( (data &  32) != 0 ) ret ^= 0x0C;
  if ( (data &  64) != 0 ) ret ^= 0x0E;
  if ( (data & 128) != 0 ) ret ^= 0x07;
  
  return ret;
}
/*****************************************************************************/
/*
 * Calculate the even parity bit for the given data byte.
 */
static byte parity ( byte data )
{
  byte ret = 0x00;

  if ( (data &   1) != 0 ) ret++;
  if ( (data &   2) != 0 ) ret++;
  if ( (data &   4) != 0 ) ret++;
  if ( (data &   8) != 0 ) ret++;
  if ( (data &  16) != 0 ) ret++;
  if ( (data &  32) != 0 ) ret++;
  if ( (data &  64) != 0 ) ret++;
  if ( (data & 128) != 0 ) ret++;

  return ret % 2;
}
/*****************************************************************************/
/*
 * Append a null terminated string to the output buffer.
 */
static void appendToOutput ( const char * pErrorMsg )
{
  while ( *pErrorMsg )
  {
    databuffer[idxWrite] = *pErrorMsg;
    idxWrite = (idxWrite+1) % RING;
    pErrorMsg++;
  }
}
/*****************************************************************************/
/*
 * Output the number of pulses during this burst.
 * used when ANALYZER mode is active.
 */
static void outputPulseCount()
{
  char s[8];
  appendToOutput ("(");
  itoa(pulse, s, 10 );
  appendToOutput ( s );
  appendToOutput (")");
}
/*****************************************************************************/
/*
 * Output the time between this and the previous burst.
 * used when ANALYZER mode is active.
 */
static void outputGapTime()
{
  char s[8];
  itoa(dt, s, 10 );
  appendToOutput ( s );
}
/*****************************************************************************/
/*
 * This handler routine is called when the comparison A interrupt of
 * timer/counter 0 occurs.
 * Counter 0 is reset at the first edge of a burst and the comparison
 * register is set to a value which covers 8 cycles of the 32768 Hz 
 * burst signal. Thus this interrupt is called when a burst is just
 * over and the time elapsed since the previous burst has been stored
 * in the variable dt by the edge detection interrupt.
 * The processing time for the idx == 14 case is the longest and takes
 * about 45 us. This still fits nicely in the gap before the next data
 * frame can be expected.
 */
ISR(TIMER0_COMPA_vect)
{
  
  #ifdef PIN_OUTPUT
    // signal start of activity
    bitSet(signalPORT, signalPin);
  #endif

  #ifdef TestPin
    digitalWrite(TestPin, HIGH);
  #endif
  

  if ( pulse < MIN_PULSES )
  {
    // If the timer elapsed and we got less than MIN_PULSES pulses, 
    // no burst is in progress.
    // This acts then as a watchdog event to make sure we are ready
    // for the next data byte, even if something went wrong
    pulse = 0;
    idx = 0;

  /*  #ifdef MODE_SWITCH
      // check mode switch
      if ( (modePORT & _BV(modePin)) == _BV(modePin) )
      {
        setMode ( MODE_INTERPRETER );
      }
      else
      {
        setMode ( MODE_ANALYZER );
      }    
    #endif
    */
  }  
  else
  {
    // we arrive here after a burst of up to 8 pulses has finished
    
    /* The index counter idx works as follows:
     *
     *       S  S  S  E  E  E  E  7  6  5  4  3  2  1  0
     *       |__|__|__|__|__|__|__|__|__|__|__|__|__|__|
     *  idx  0  1  2  3  4  5  6  7  8  9 10 12 13 14 15
     *
     */        
    if ( idx == 0 )
    {
      // this is the start of first burst
      if ( mode == MODE_ANALYZER )
      {
        // output the number of pulses during this burst
        outputPulseCount();
      }
    }
    else 
    {
      // this is the start of the second or later burst
      
      //-------------------
      if ( mode == MODE_ANALYZER )
      {
        // output the time interval between bursts
        outputGapTime();
        
        // output the number of pulses during this burst
        outputPulseCount();
      }
      //-------------------
  
      
      if ( idx == 1 )
      {
        // second start half-bit
        // gap 1 after second burst
        halfBitTicks = dt;
      }
      else if ( idx == 2 )
      {
        // third start half-bit
        // gap 2 after third burst
        // take the average of the two gaps between the three start half-bits
        halfBitTicks = (halfBitTicks + dt) >> 1;
      }
      else
      {
        // other gaps
        // use static variables as we return here 12 times to compose the
        // 4 ECC bits and the 8 data bits step by step
        static byte lastbit = 0;
        static byte token   = 0;
        static byte missed  = 0;
        static byte ECC     = 0;
        static byte data    = 0;
  
        // ECC:  idx=3,4,5,6
        // data: idx=7,8,9,10,11,12,13,14
        
        if ( idx == 3 )
        {
          // start of the error correction nibble
          // set lastbit from last start half-bit which equals a zero
          lastbit = 0;
          // reset bit collector
          token = 0;
          // start with no missed bits in ECC nibble
          missed = 0;
        }
        else if ( idx == 7 )
        {
          // start of the data byte
          // we carry lastbit over from ECC
          // reset bit collector
          token = 0;
          // start with no missed bits in data byte
          missed = 0;
        }
  
        // determine number of 1/2-bit intervals since last burst (rounded up)
        int t = (dt + halfBitTicks/4) / halfBitTicks;
  
        if ( lastbit == 1 )
        {
          // we have to adjust by one interval to match the correct
          // interval in the following switch (t)
          t--;
        }
        
        // here we detect the bit sequence for this interval
        // in case of missed bits we remember this in the 
        // missed bit mask and also increment the index idx.
        // this could be written more compact, but I preferred a readable form
        switch ( t )
        {
          case 0:
            // second burst in same bit: this is an error which we ignore
            break;
            
          case 1:          // one bit '1'
            lastbit = 1;
            token <<= 1;   // slot for new bit
            token |= lastbit;    // set '1'
            missed <<= 1;  // no missed bit, just move mast to the left
            break;
            
          case 2:          // one bit '0'
            lastbit = 0;
            token <<= 1;   // slot for new bit
            // nothing to set for '0'
            missed <<= 1;  // no missed bit, just move mast to the left
            break;
            
          case 3:             // one missed bit, one bit '1'
            lastbit = 1;
            token <<= 1;      // skip missed bit
            token <<= 1;      // slot for new bit
            token |= lastbit;       // set '1'
            missed <<= 1;     // move mask to the left
            missed |= 1;      // insert one missed bit
            idx++;            // account for missed bit in bit count 
            break;
            
          case 4:           // one missed bit, one bit '0'
            lastbit = 0;
            token <<= 1;   // skip missed bit
            token <<= 1;   // slot for new (null) bit
            // nothing to set for '0'
            missed <<= 1;
            missed |= 1;      // insert one missed bit
            idx++;            // account for missed bit in bit count 
            break;
            
          case 5:         // two missed bits, one bit '1'
            lastbit = 1;            
            token <<= 1; // missed bit
            token <<= 1; // missed bit
            token <<= 1; // slot for new bit
            token |= lastbit;  // set '1'
            missed <<= 1;
            missed |= 1;
            idx++;            // account for missed bit in bit count 
            missed <<= 1;
            missed |= 1;
            idx++;            // account for missed bit in bit count 
            break;
            
          case 6:         // two missed bits, one bit '0'
            lastbit = 0;
            token <<= 1; // missed bit
            token <<= 1; // missed bit
            token <<= 1; // slot for new (null) bit
            // nothing to set for '0'
            missed <<= 1;
            missed |= 1;
            idx++;            // account for missed bit in bit count 
            missed <<= 1;
            missed |= 1;
            idx++;            // account for missed bit in bit count
            break;
            
            // anything else would be an error which we ignore
        }
  
        
        if ( idx == 6 )
        {
          // we have captured all 4 error correction bits
          // note: ECC:  idx=3,4,5,6
          if ( missed )
          {
            // missed bit(s) in ECC nibble
            // it does not make sense to try a correction of the data byte
            ECC = 0;
          }
          else
          {
            // save for possible later correction of the data byte
            ECC = token;
          }
        }
        else if ( idx == 14 )
        {
          // we have captured all 8 data bits
          // note: data: idx=7,8,9,10,11,12,13,14

          // blink on LED whenever there is data in the buffer   

          #ifdef outsideLED        
            digitalWrite(outsideLED, !digitalRead(outsideLED));
          #endif
          
          if ( missed > 0 )
          {
            // there are more than three missed bit(s) in data byte
            // error message shows missed bits as a binary mask
            char s[8];
            appendToOutput ( "{M=" );
            itoa( missed, s, 2 );
            appendToOutput ( s );
            appendToOutput ( "}" );
          }
          
          //byte checkECC = calcECBits ( token );

          // correction of missed bits          
          if ( ECC != 0 && missed > 0 && missed < 3 )
          {
            // ECC nibble is o.k. and we have one or two missed bits
            
            //static const byte H[] = { 0x78, 0xE6, 0xD5, 0x8B };
            static const byte H[] = { 0b01111000,
                                      0b11100110,
                                      0b11010101,
                                      0b10001011 };
                                      
            while ( missed != 0x00 )
            {
              for ( byte i=0 ; i<4 ; i++ )
              {
                // handle the four ECC bits from left to right
                byte mask = H[i];
                byte x = missed & mask;

                if ( parity( x ) == 1 )
                {
                  // this mask has exactly one bit in common with the
                  // missed bit mask
                  if ( parity ( token & mask ) != ((ECC>>(3-i)) & 0x01) )
                  {
                    // set bit
                    token |= x;
                  }
                  // else
                  // { bit stays empty }
				
                  // nullify missed bit
                  missed &= ~x;
                  // continue with next missed bit (if any)
                  break;
                }			
              }
            }
          } // correction of missed bits
          
          
          //-------------------
          if ( mode == MODE_ANALYZER )
          {
            // output the half bit time for information
            char s[8];
            appendToOutput ( " = '" );
            s[0] = token > 31 ? (token < 128 ? token : '.') : '.';
            s[1] = '\0';
            appendToOutput ( s );
            appendToOutput ( "' = 0x" );
            if ( token < 16 ) appendToOutput ( "0" );
            itoa(token, s, 16 );
            appendToOutput ( s );
            appendToOutput ( ", 1/2-bit = " );
            itoa(halfBitTicks, s, 10 );
            appendToOutput ( s );
            appendToOutput ( "\n" );
          }
          //-------------------

          if ( mode != MODE_ANALYZER )
          {
            databuffer[idxWrite] = token;
            idxWrite = (idxWrite+1) % RING;
          }         
          
          // the last burst is over
          // reset index for storing timing of next byte
          // will be incremented to 0 below
          idx = -1;
        } // idx == 14
      } // idx > 2
    } // idx > 0
    
    // burst is over, prepare for next bit
    idx++;

    // prepare for next burst
    pulse = 0;
  }

  #ifdef PIN_OUTPUT
    // set output pin to low to signal end of activity
    bitClear(signalPORT, signalPin);
  #endif

  #ifdef TestPin
    digitalWrite(TestPin, LOW);
  #endif
  
}
/*****************************************************************************/
/*
 * This interrupt function is called when the input pin with the IR sensor 
 * drops (negative logic).
 * This indicates the start of a cycle of an IR-burst.
 */
#if INT_NO==INT1
  ISR(INT1_vect)
#endif
#if INT_NO==INT6
  ISR(INT6_vect)
#endif

{
  // check for the first pulse and increment the pulse count
  if ( pulse++ == 0 )
  {
    // this is the first pulse of a burst    

    // save time since start of last burst
    dt = TCNT0;
    
    // restart the timer to measure the time to the next burst
    TCNT0 = 0;
    
    // enable output compare A interrupt
    TIMSK0 = (1<<OCIE0A);
    
    // Now a burst is in progress. The index idx will be increased after 
    // the burst is over in the TIMER0_COMPA_vect service routine.
  }
}
/*****************************************************************************/

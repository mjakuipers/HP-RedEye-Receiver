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
 * 
 ************************************************************************
 * 
 * Purpose of this code
 * ====================
 * This software implements a decoder for the HP 82240A/B infrared
 * printer protocol as used by Hewlett Packard pocket calculators of 
 * the 1980/90s (e.g. the HP 28C and S as well as the HP 48 family).
 * The protocol, also known as the "Red Eye" protocol was a precursor to
 * the IrDA standard.
 * Some more recent HP pocket calculators still support this protocol but
 * favor the IrDA standard before dropping IR links for wired RS232 and
 * USB connections.
 * The "Red Eye" protocol is also implemented in the WP34S, a repurposed
 * HP 20b/30b pocket calculator.
 *
 * Suitable Hardware
 * =================
 * This software requires a receiver hardware which captures infrared
 * signals sent from a suitable calculator.
 * The decoding is done by this software using facilities of an Amtel 
 * AVR chip, with at least one interrupt and one timer.
 * The output of the decoder is a serial byte stream which is sent via 
 * a serial connection to a PC or any other device having a serial interface.
 * It is possible to use a serial-to-USB converter cable to simulate a COM
 * port via USB.
 * One could also easily adapt the code to support a parallel printer.
 *
 * The processing hardware can be a small Arduino (I used a Leonardo, but 
 * a smaller variant of AVR chip would be sufficient) plus an IR receiver
 * connected to an input pin of the microcontroller.
 *
 * The IR receiver must be able to detect individual pulses sent with a 
 * frequency of 32.768 kHz.
 * Therefore integrating receivers like the TSOP/VS family and similar are
 * NOT suited for this purpose. The problem with these modern (2014)
 * integrating receivers is that they require a pause between bursts which
 * is longer than the pause provided by the "Red Eye" protocol.
 * Therefore I used a HP C4103A receiver unit, which was originally provided
 * for HP laser printers.
 *
 * The range with a HP-48G is more than 1 meter and very robust.
 * With more recent HP calculators and the WP-34S the maximum distance is
 * much shorter, more something like 100 mm or even less.
 *
 * Receiving Software
 * ==================
 * The byte stream contains escape sequences which control the printer,
 * for example to switch to graphics mode and back.
 * One means to interpret this stream is Christoph Gießelinks excellent
 * HP 82240B printer simulator program on Windows, which can read its 
 * input from a COM port.
 *
 * For other platforms the command line printer simulator from Khanh-Dang 
 * Nguyen Thu-Lam works well. I tried it on Windows and only a tiny
 * modification was required to capture incoming text and graphics.
 * As pure C-code this tool will also work on all other platforms which
 * have a C compiler (I used gcc).
 *
 * This Software
 * =============
 * Besides the simple pass-through mode the code also implements a simple
 * interpreter with a state machine for control codes of the printer.
 * This could be modified to drive any printer or perform other tasks, 
 * like controlling devices or whatever you like.
 *
 * All printer control sequences are decoded but in the current 
 * implementation no special action is taken. 
 * Bitmap graphics are also recognized and stored for further processing.
 *
 * Sending the command characters A(nalyzer), I(nterpreter), T(ranslate) via
 * the serial (USB) link allows to activale some special modes:
 * 
 * The Interpreter mode is the normal operating mode. It passes all data
 * straight through.
 *
 * The Analyzer mode creates output formatted like this:
 * (8)27(8)26(8)54(8)27(8)53(8)80(8)53(8)54(8)53(8)54(8)53(8)27(8)80(8)53(8)
 *      D = 'A' = 0x41, 1/2-bit = 26
 * The numbers in parentheses indicate the number of pulses per burst
 * and the numbers between the bursts specify the gap time.
 * D = is the data byte recognized and at the end the half bit time
 * derived from the start half-bits is given.
 *
 * The Translate option switches between passing through and replacing the
 * character code 0x04 by a linefeed character. This allows to view the 
 * print stream of some calculators in a terminal (emulator).
 *
 *
 *
 * Some comments on the basic architecture:
 * ========================================
 * In order to achieve a smooth operation, one interrupt and one timer is used:
 * - The interrupt is used to capture the start of each pulse (when the
 *   input pin goes low, as my IR sensor has an inverted logic). 
 *   Thus this interrupt counts all pulses within each burst.
 *   It flags the first pulse of each burst and determines the time between 
 *   subsequent bursts.
 *
 * - Timer 0 is started to trigger just after the end of each burst
 *   assuming the maximum nominal length of 8 pulses.
 *   When the timer is triggered, the time to the previous bit is evaluated 
 *   and the appropriate bit value is set. There is enough time after each
 *   burst to do this processing.
 *   The timer also resets the pulse count to be ready for the next burst.
 *
 * - The error checking nibble and the data byte are thus composed bit by bit.
 *
 * - After the last bit has been decoded, the resulting byte is put into a
 *   ring buffer.
 *
 * Any post-processing of the data in the ring buffer (if desired) can be
 * performed in the main loop. This may be interrupted at any time when
 * new data arrives.
 * This setup yields a very reliable system, even if larger amounts of data
 * are sent. Such larger amounts of data occur when graphics, e.g. a screen
 * copy, is sent.
 *
 ************************************************************************
 *
 * "Red Eye" infra red signal (for more details see references [1,2])
 *
 * The following sketch shows the usage of interrupt and timer:
 * The interrupt INT is triggered on falling edge of each pulse
 *
 *   INT on first pulse starts TIMER0
 *      |-------------TIMER0-------------------->
 *                                              | 
 *                                              | TIMER0 elapses
 *                                              | after 8+ bursts
 *     INT  INT  INT  INT  INT  INT  INT  INT   |
 *      |    |    |    |    |    |    |    |    |
 *      v    v    v    v    v    v    v    v    v
 * -----+ +--+ +--+ +--+ +--+ +--+ +--+ +--+ +----------------
 *      | |  | |  | |  | |  | |  | |  | |  | |
 *      +-+  +-+  +-+  +-+  +-+  +-+  +-+  +-+
 *      1    2    3    4    5    6    7    8 pulses 
 *                                           @ 32.768 kHz
 *      <------------- one burst ------------->
 *
 *
 * The following sketch shows the encoding of the 0 and 1 bits:
 * (bursts are indicated by the three vertical lines) 
 *
 *    burst       burst             burst       burst burst        
 *      |           |                 |           |     |  
 *      v           v                 v           v     v
 *      |  logic 1  |  logic 1  |  logic 0  |  logic 0  |  logic 1  |
 *      |           |           |           |           |           |
 *      |  .  |  .  |  .  |  .  |     |     |     |     |     |     |
 * -----|||---------|||---------------|||---------|||---|||---------
 *      |||         |||               |||         |||   |||
 *      .     .     .
 *      <====>      .
 *    one half-bit = 427.25 microseconds
 *      .           .
 *      <===========>
 *         one bit = 854.5 microseconds
 * 
 * Each bit takes 854.5 microseconds (28 cycles of a 32768 Hz clock of the
 * IR driver).
 * The shortest spacing occurs within the three start bits which are are
 * spaced 854.5/2 = 427.25 microseconds apart.
 * A burst at the start of the FIRST  half-bit represents a logic '1'
 * A burst at the start of the SECOND half-bit represents a logic '0'
 * IMPORTANT:
 * Per specification the burst time is 6-8 cycles of the 32786 Hz clock,
 * which amounts to 183-244 microseconds (183 = 1/4.67 of the bit time).
 * The number of pulses in each burst can vary between 6 and 8.
 * Some calculators (e.g. the HP 28S) adapt the number of pulses to
 * achieve a "good" average timing, which, despite these efforts is less
 * accurate than e.g. the signals produced by machines which use a crystal
 * for the inter-bit timing.
 *
 ************************************************************************
 *
 * References
 * [1] Harper, Worsley, Stephens, "An Infrared Link for Low-Cost
 *        Calculators and Printers", Hewlett-Packard Journal, October 1987.
 * [2] Anonymous, "HP 82240V Infrared Printer Technical Interfacing Guide".
 * [3] Steve ?. "IR Printer Information"", The Museum of HP Calculators, 
 *        HP Articles Forum, 31 July 2000, [web site]
 * [4] "Hamming Error Correction Code", robot room, [web site].
 * [5] "Error Detection Correction", supplement to Logic and Computer 
 *        Design Fundamentals, 3rd Edition, 2004.
 * [6] Christoph Gießelink, "redeye.c" source code from EMU28 project, 2011 .
 *
 *
 ************************************************************************
 */
// see redeye.cpp
#include "RedEye.h"

/*
 * I have implemented an interpreter with limited postprocessing.
 * This can be expanded to interface to any printer or other device.
 * This implementation covers only the basic skeleton - a more complete
 * implementation is left as an exercise to the user.
 */

// state codes for interpreter
#define TEXT        0
#define GRAPHICS    1
// character codes
#define ALT_CR      4  // char 4 ==  carriage return/linefeed for 82440 printer
#define LF         10  // char 10
#define ESC        27  // char 27 0x1B

// 115200 works fine for modern PCs with USB-Serial
// 9600 works fine for old PCs with Serial
#define BAUD_RATE 115200

  #define hi_val 250
  #define lo_val 10
  #define dimstep 1

  int countup = 0;   // counting up = 0, counting dow <> 0
  int dim = lo_val;       // value for PWM

  unsigned long cur_time;
  unsigned long pre_time = micros();
  long int_time = 1000;    // 1000 useconds

// state of the machine
int state = TEXT;
// for collecting a line of textual respectively graphics bytes
// (must be at least 166 bytes long)
#define BUFFER_SIZE 200
byte lineBuffer[BUFFER_SIZE];
// counter for textual bytes
int idxLineBuffer;
// special counter for graphics bytes
int numGraphicsBytes = 0;
// is true: translate alternate CR 0x04 to linefeed 0x10
boolean translateAltCR = false;

/*****************************************************************************/
/*
 * This is the usual Arduino style setup() function which is executed
 * only once.
 */
void setup()
{
  
  // Prepare the serial output port.
  // You can connect Christoph Gießelinks printer simulator to the
  // appropriate COM port to receive and save the printout on a Windows PC.
  // note that this only supports baud rates up tp 9600 baud
  Serial.begin(BAUD_RATE);

  #ifdef outsideLED
    // switch on outside LED and blink
    // makes up for the delay to make Serial raedy
    pinMode(outsideLED, OUTPUT);
    digitalWrite(outsideLED, LOW);
    delay(250);
    digitalWrite(outsideLED, HIGH);
    delay(250);
    digitalWrite(outsideLED, LOW);
    delay(250);
    digitalWrite(outsideLED, HIGH);
    delay(250);
    digitalWrite(outsideLED, LOW);
    delay(250);
    digitalWrite(outsideLED, HIGH);
    delay(250);
    digitalWrite(outsideLED, LOW);
    delay(250);
    digitalWrite(outsideLED, HIGH);
    delay(250);
    digitalWrite(outsideLED, LOW);
    delay(250);
    digitalWrite(outsideLED, HIGH);
    delay(250);
    digitalWrite(outsideLED, LOW);
    delay(250);
    digitalWrite(outsideLED, HIGH);    
  #endif
    
  #ifndef outsideLED
    // we should make sure that Serial is ready. Possibly unnecessary,
    // but cannot hurt.      
    // delay(2500);  
  #endif
  
  // here comes the heavy stuff...
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);  
  pre_time = micros();

  setupRedEye();
}

/*****************************************************************************/
/*
 * This is the usual Arduino style main() loop.
 */
/*****************************************************************************/
void loop()
{  
  // we twiddle our thumbs here - but at full speed
  // to conserve energy and to save the planet, one 
  // could also reduce processor speed and speed it
  // up again whenever data is received.

  // changes by Meindert Kuipers to the mode settings
  // A - activates Analyzer mode
  // I - activates Interpreter mode (data is buffered for HP and interpreted)
  // P - activates Passthrough mode
  // the modes above are mutually exclusive. Passthrough mode is default
  // T - enables CR translation. 
  //     default is off (no translation)

  int m = getMode();

  
  while ( true )
  {

    // this is the place to PWM the LED showing a hartbeat
    // by Meindert Kuipers
    cur_time = micros();
    if (cur_time - pre_time > int_time) {
      pre_time = cur_time;
      digitalWrite(9, !digitalRead(9));
      // more than 1000 useconds have passed, so adjust PWM
      if (dim > hi_val) { countup = -1; }         // now count down
      if (dim < lo_val) { countup =  0; }         // now count up

      if (countup == 0) {
        // counting up
        dim = dim + dimstep;
      } else {
        // counting down
        dim = dim - dimstep;
      }
      analogWrite(outsideLED, dim);
      
    }
    

    if ( isDataAvailable() )
    {
      
      // blink on LED whenever there is data in the buffer           
      dim = lo_val;
                
      // there is something in the receiving buffer      
      // avoid that reading and updating idxRead is interrupted
      // pull the byte from the buffer ...
      byte data = getByte();

      m = getMode();

      if ( true )
      {
        // PASSTHROUGH mode
        // just pass the raw data
        Serial.write(data);
      }
      else
      {
        // this is our interpreter state machine
        
        switch ( state )
        {

          //----------------
          case ESC:
          //----------------
            // we are in ESCape state (previous character was ESC / 27d / 0x1B)
            
            // switch to TEXT state (may be overwritten below)
            state = TEXT;
            
            switch ( data )
            {
              // ESC control codes
              // we just swallow the following codes
              case 255:  reset();
              case 254:  /* "SELF TEST"         */
              case 253:  /* "DOUBLE WIDE"       */
              case 252:  /* "SINGLE WIDE"       */
              case 251:  /* "UNDERLINE"         */
              case 250:  /* "NO UNDERLINE"      */
              case 249:  /* "ISO 8859 Latin 1"  */
              case 248:  /* "Roman8"            */
                // pass through
                addToBuffer(ESC);
                addToBuffer(data);
                flush();
                break;
                
              /* 
               * we can define our own escape sequences like 
               * ESC 0 or ESC (167 to 247).
               * ESC 1 ... 166 is blocked by graphics.
               * One could also enter another mode e.g. by ESC 0, read some
               * commands and then switch back with another ESC 0
               */
              case   0:
                // ESC 0
                // swallow ESC and zero
                break;
        
              default:
                if ( data > 0 && data < 166+1 )  
                {
                  // within allowable range for graphics width (1...166)
                  // empty current buffer and prepare for new graphics bytes
                  flush();                 
                  // # of graphics bytes to follow
                  numGraphicsBytes = data;
                  // switch to GRAPHICS state
                  state = GRAPHICS;
                }
                else
                {
                  Serial.print("UNKNOWN ESC ");
                  Serial.println(data,DEC);
                  addToBuffer(ESC);
                  addToBuffer(data);
                }
                break;
            }
            
            break;
            
          //----------------
          case GRAPHICS:
          //----------------
            // store the next graphics byte
            lineBuffer[idxLineBuffer++] = data;          
            if ( idxLineBuffer == numGraphicsBytes )
            {
              // graphics sequence is complete
              // flush buffer to insert proper graphics ESCape prefix
              flush();
              // revert to normal text mode
              state = TEXT;
            }
            break;
            
            
          //----------------
          default:
          //----------------
            // not in ESCape state, not in GRAPHICS state, therefore TEXT state
            switch ( data )
            {
              case ESC: // 27: escape
                // do not output the escape character now
                state = ESC;
                break;
                
              case ALT_CR: // 0x04: alternate line feed,
                       // is generally used  by HP 48, HP 28C
                if ( translateAltCR )
                {
                  data = LF;  // replace
                }
                // fall trhough
              case LF: // 0x0A: linefeed
                addToBuffer(data);
                // ship out content of printer buffer and prepare for new data
                flush();
                break;
                
              default:
                // store data in printer buffer
                addToBuffer(data);
                break;
            }
            break;    
        }
      }
    }
    
    // check for input
    while ( Serial.available() > 0 )
    {
      int b = Serial.read();

      switch ( b )
      {
        case 'A':
          // change mode
          m = getMode();
          Serial.print(m, DEC);          
          setMode ( MODE_ANALYZER );
          m = getMode();
          Serial.print(m, DEC);
          Serial.print("=>MODE_ANALYZER\n");
          break;
          
        case 'I':
          // change mode
          m = getMode();
          Serial.print(m, DEC);
          setMode ( MODE_INTERPRETER );
          m = getMode();
          Serial.print(m, DEC);
          Serial.print("=>MODE_INTERPRETER\n");
          break;

        case 'P':
          // change mode
          m = getMode();
          Serial.print(m, DEC);
          setMode ( MODE_PASSTHROUGH );
          m = getMode();
          Serial.print(m, DEC);
          Serial.print("=>MODE_PASSTHROUGH\n");
          break;
          
        case 'T':
          translateAltCR = !translateAltCR;
          Serial.print("=>CR-translation: ");
          Serial.print(translateAltCR ? "ON" : "OFF");
          Serial.print("\n");
          break;
      }
    }
    
  } // while ( true )
}
/*****************************************************************************/
void addToBuffer( byte data )
{
  // perform possible modification here
  lineBuffer[idxLineBuffer++] = data;
  
  // check for line overflow (should never occur)
  if ( idxLineBuffer == BUFFER_SIZE )
  {
    flush();
  }
}
/*****************************************************************************/
/*
 * Reset the printer.
 *
 */
void reset()
{
  idxLineBuffer = 0;
  numGraphicsBytes = 0;
  Serial.write(0x04);
}
/*****************************************************************************/
/*
 * Flush the printer buffer.
 *
 */
void flush()
{
  if ( idxLineBuffer > 0 )
  {
    if ( numGraphicsBytes > 0 && numGraphicsBytes == idxLineBuffer )
    {
      // there is some GRAPHICS in the buffer
      
      // send the graphics buffer with proper ESC+COUNT prefix
      Serial.write(ESC);
      Serial.write(numGraphicsBytes);
#ifdef FUN
      // just for fun: invert image
      for ( int i=0 ; i<numGraphicsBytes ; i++ )
      {
        lineBuffer[i] = ~lineBuffer[i];
      }
#endif
      Serial.write(lineBuffer,numGraphicsBytes);
      // reset count
      numGraphicsBytes = 0;
    }
    else
    {
      // there is some TEXT in buffer
      
      // send the graphics buffer with proper ESC+COUNT prefix
      Serial.write(lineBuffer,idxLineBuffer);
    }    
    idxLineBuffer = 0;
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

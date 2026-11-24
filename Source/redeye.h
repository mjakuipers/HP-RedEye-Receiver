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
  *
 * Version modified by Meindert Kuipers to support the Arduino Pro Micro
 * adds support for controlling the LED on the C4103A housing with pin D5
 * Uses pin D3 for the IRQ output for testing
 * 
 ************************************************************************
 */

#include <Arduino.h>

/* Adapt pin and port as needed
 *
 * Here we use interrupt 1 or interrupt 6
 *
 * Example: interrupt 1:
 * In the ATmega32U4 chip this is connected to pin PD1 (pin 1 of port D).
 * On the Arduino Leonardo board this is connected to digital Pin D2.
 * So we connect the wire to Pin 2, but in the code we have to use 
 * the ATmega parameters for "input pins, port D" (PIND) and pin 1 (PD1).
 *
 * If we want to use SDA and SCL on pins D2, D3 so that we cannot use INT1.
 * Then we can use INT6 on pin D7
 */

#define INT_NO    INT1      // Pro Mini and Pro Micro
//#define INT_NO    INT6

// output LED on housing
#define outsideLED 5   // digital output pin 5, can also be used for PWM

// define TEST_PIN, become high when in Timer ISR
#define PIN_TEST

// define for using the heartbeat LED with PWM, just because we can
#define HEARTBEAT

/* 
 * If we use a Pro Mini the pin mapping has to be adapted
 */

// #define LEONARDO
// #define PRO_MINI              // actually using Pro Micro!
#define PRO_MICRO

/****************************************************/
#define MODE_INTERPRETER 1
#define MODE_ANALYZER    2
#define MODE_PASSTHROUGH 3
/****************************************************/

// global functions
extern void setupRedEye();
extern boolean isDataAvailable();
extern byte getByte();
extern boolean isInterpreter();
extern void setMode ( int );
extern int getMode();
extern void dimmer_int();

/****************************************************/

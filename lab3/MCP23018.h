/*
  Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>
 
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.
*/
#pragma once

#include <Wire.h>

// MCP23018 registers

#define IODIRA 0x0
#define IODIRB 0x1
#define IPOLA 0x2
#define IPOLB 0x3
#define INTENA 0x4
#define INTENB 0x5
#define DEFVALA 0x6
#define DEFVALB 0x7
#define INTCONA 0x8
#define INTCONB 0x9
#define IOCON 0x0A
#define GPPUA 0x0C
#define GPPUB 0x0D
#define INTFA 0x0E
#define INTFB 0x0F
#define INTCAPA 0x10
#define INTCAPB 0x11
#define GPIOA 0x12
#define GPIOB 0x13
#define OLATA 0x14
#define OLATB 0x15

#define IOCON_BANK 0x80
#define IOCON_MIRROR 0x40
#define IOCON_SEQOP 0x20
#define IOCON_ODR 0x04
#define IOCON_INTPOL 0x02
#define IOCON_INTCC 0x01

#define MCP23018_ADDR 0x20

/**
 * Driver for Microchip MCP23018 Port Extender
 *
 * See <a href="http://ww1.microchip.com/downloads/en/DeviceDoc/22103a.pdf">Datasheet</a>
 *
 * Create one instance of this class for each chip in your circuit.
 *
 */
class MCP23018
{
protected:
	/**
	 * The 7-bit address of the device we're driving on the I2C bus.
	 */
	uint8_t i2c_address;

public:
	/**
	 * Constructor
	 *
	 * @param _address the low 3 bits of the address for the chip being
	 * addressed.  This should correspond to the voltage on the ADDR pin.
	 */
	MCP23018(uint8_t _address);

	/**
	 * Setup & prepare
	 *
	 * Call this function from within the setup() function of your
	 * sketch, AFTER Wire.begin().
	 *
	 * @warning This currently sets both ports to be outputs, which is
	 * not very general-purpose.
	 */
	int begin();

	/**
	 * Write twice to a control register
	 *
	 * I think this is only useful for the GPIOA/B registers, which automatically
	 * toggle between themselves after writing one.
	 *
	 * @param address The address of the particular register
	 * @param first The first byte to write into the particular register
	 * @param second The second byte to write into the particular register
	 */
	int writePairToRegister(uint8_t address, uint8_t first, uint8_t second);

	/**
	 * Read an entire control register
	 *
	 * @param address The address of the particular register
	 * @return The data read from the particular register
	 */
	int readFromRegister(uint8_t address);
	int readPairFromRegister(uint8_t address);

	/**
	 * Write an entire control register
	 *
	 * @param address The address of the particular register
	 * @param data The data to write into the particular register
	 */
	int writeToRegister(uint8_t address, uint8_t data);

	/**
	 * Set a single bit in a register
	 *
	 * This loads all the values from the device first, sets the correct bit
	 * an pushes them back.
	 *
	 * @param address_bit The address and bit number (0-7)
	 * @param bitState Whether to set the bit (true) or clear it (false)
	 */
	int setBitInRegister(uint8_t addr, uint8_t bit, bool state);
	int setMaskInRegister(uint8_t addr, uint8_t mask, bool state);

	int SetDirections(uint8_t _a, uint8_t _b);
	/**
	 * Set the pull-up resistors
	 *
	 * The MCP23018 has open-drain inputs, which means if you want to use the
	 * outputs to drive logic inputs on some other device, you need to engage
	 * the pullup resistors.
	 *
	 * @param _a One bit for each line on port 'A', send 1 to enable the pullup, 
	 * 0 to disable
	 * @param _b One bit for each line on port 'B', send 1 to enable the pullup, 
	 * 0 to disable
	 */	
	int SetPullups(uint8_t _a, uint8_t _b);

	/**
	 * Set the outputs on Port A
	 *
	 * @param _data Bits to set
	 */	 
	int SetPortA(uint8_t _data);
	
	/**
	 * Set the outputs on Port B
	 *
	 * @param _data Bits to set
	 */	 
	int SetPortB(uint8_t _data);

	/**
	 * Get the input values on Port A
	 *
	 * @return The bits set on Port A inputs 
	 */	 
	int GetPortA();
	
	/**
	 * Get the input values on Port B
	 *
	 * @sa GetPortA
	 * @return The bits set on Port B inputs 
	 */	 
	int GetPortB();
	
	/**
	 * Get the output latches on Port A
	 *
	 * The latches are the values we previously set, rather than trying to read the
	 * port itself.  If you want to read the previously-set values with an eye toward
	 * updating only certain bits, use this method.
	 * 
	 * @return The bits set on Port A 
	 */	 
	int GetLatchPortA();
	
	/**
	 * Get the output latches on Port B
	 *
	 * @sa GetLatchPortA 
	 * @return The bits set on Port B 
	 */
	int GetLatchPortB();
	
  /* set a pin to high or low depending on state */
	int SetAPin(uint8_t pin, bool state);
	int SetBPin(uint8_t pin, bool state);
};

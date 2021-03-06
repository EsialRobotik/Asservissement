/*
 * EcouteI2c.h
 *
 *  Created on: 13 avr. 2017
 *      Author: pmx
 */

#if CONFIG_COM_I2C_ACTIVATE

#ifndef ASSERV_MAIN_ECOUTEI2C_H_
#define ASSERV_MAIN_ECOUTEI2C_H_
class CommandManager;
class ConsignController;
class MotorsController;
class Odometrie;
#include "Main.h"

#define I2C_SLAVE_ADDRESS 0x05

int code = 0;
int nbdata = 0;

// convert float to byte array  source: http://mbed.org/forum/helloworld/topic/2053/
union float2bytes_t// union consists of one variable represented in a number of different ways
{
    float f;
    unsigned char bytes[sizeof(float)];
};

void ecouteI2c(ConsignController *consign, CommandManager *cmd, MotorsController *mot, Odometrie *odo, bool *run);
void ecouteI2cConfig();

#endif /* ASSERV_MAIN_ECOUTEI2C_H_ */

#endif /* CONFIG_COM_I2C_ACTIVATE */

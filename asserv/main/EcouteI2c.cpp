/*
 * EcouteI2c.cpp
 *
 *  Created on: 13 avr. 2017
 *      Author: pmx
 */

#include "EcouteI2c.h"

#include <DigitalOut.h>
#include <I2CSlave.h>
#include <PinNames.h>
#include <wait_api.h>
#include <cstdio>

#include "../motorsController/Md25/Md25ctrl.h"
#include "../odometrie/Odometrie.h"
#include "../Utils/Utils.h"
#include "Main.h"

//Definitions globales
#ifdef COM_I2C_ACTIVATE
I2CSlave slave(p9, p10);
#endif

#ifdef COM_I2C_ACTIVATE
void ecouteI2cConfig()
{
	slave.frequency(400000);
	slave.address(I2C_SLAVE_ADDRESS << 1); //We shift it left because mbed takes in 8 bit addreses

}

//Name		Permissions	DataAddress	ReadResponse	WriteAction
//Whoami		R		0x00		Device ID		N/A
//setPosition	W		S			N/A				X Y T
//getPosition	R		P			x y t			N/A

//Control1		RW		0x01		Actual Value	Set Setpoint
//Control2		RW		0x02		Actual Value	Set Setpoint
//Sensor		R		0x03		Sensor Data		N/A

int code = 0;
int nbdata = 0;

void ecouteI2c(ConsignController *consign, CommandManager *command, MotorsController *mot,
		Odometrie *odo)
{

	char buf[2]; //I2C_Packet and PACKET_SIZE
	char cmd[12];
	char ack[1];
	ack[0] = I2C_SLAVE_ADDRESS;
	int r = 0;

	int i = slave.receive();
	switch (i)
	{
	case I2CSlave::ReadAddressed:

		if (code == 0)
		{
			slave.write(ack, 1);
#ifdef DEBUG_COM_I2C
			printf("\r\nI2CSlave::ReadAddressed ACK\r\n");
#endif
		}

		if (code == 'P')
		{
			if (nbdata != sizeof(cmd))
			{
				printf("ERROR I2CSlave::ReadAddressed (code=%c) ; bad nbdata and sizeof(cmd) !\r\n",
						code);
				ErrorLed = 1;
			}
			else
			{
				gotoLed = !gotoLed;

				float2bytes_t x;
				x.f = odo->getXmm();
				cmd[0] = x.bytes[0];
				cmd[1] = x.bytes[1];
				cmd[2] = x.bytes[2];
				cmd[3] = x.bytes[3];
				//printf("      Write : %d %d %d %d\r\n", cmd[0], cmd[1], cmd[2], cmd[3]);
				float2bytes_t y;
				y.f = odo->getYmm();
				cmd[4] = y.bytes[0];
				cmd[5] = y.bytes[1];
				cmd[6] = y.bytes[2];
				cmd[7] = y.bytes[3];
				//printf("      Write : %d %d %d %d\r\n", cmd[4], cmd[5], cmd[6], cmd[7]);
				float2bytes_t t;
				t.f = odo->getTheta();
				cmd[8] = t.bytes[0];
				cmd[9] = t.bytes[1];
				cmd[10] = t.bytes[2];
				cmd[11] = t.bytes[3];
				//printf("      Write : %d %d %d %d\r\n", cmd[8], cmd[9], cmd[10], cmd[11]);
				r = slave.write(cmd, sizeof(cmd));
				if (r == 0)
				{
#ifdef DEBUG_COM_I2C
					printf("P12 x=%f  y=%f  t=%f\r\n", x.f, y.f, t.f);
#endif
				}
				else
				{
					printf(
							"ERROR I2CSlave::ReadAddressed (code=%c) : impossible to send position !\r\n",
							code);
					ErrorLed = 1;
				}
			}

			code = 0;
			nbdata = 0;
		}

		break;

	case I2CSlave::WriteGeneral:
		//slave.read(buf, sizeof(buf));
		//printf("Read G: %s\r\n", buf);
		break;

	case I2CSlave::WriteAddressed:

		if (code == 0)
		{
			r = slave.read(buf, 2); //2 bytes => 0.5ms

			if (r == 0)					//uniquement si ok.
			{
#ifdef DEBUG_COM_I2C
				//printf("I2CSlave::WriteAddressed: %c%d\r\n", buf[0], (int) buf[1]);
#endif
				code = buf[0];
				nbdata = (int) buf[1];
			}
			else
			{
				code = 0;
				nbdata = 0;
				printf("ERROR I2CSlave::WriteAddressed: IMPOSSIBLE TO READ FIRST COMMAND!\r\n");
				ErrorLed = 1;
			}
		}
		else
		{
			if (code == 'S') //set position
			{
				if (nbdata != sizeof(cmd))
				{
					printf("ERROR I2CSlave::WriteAddressed (code=%c) : nbdata != sizeof(cmd) !\r\n",
							code);
					ErrorLed = 1;
				}
				else
				{
					r = slave.read(cmd, sizeof(cmd)); //12 bytes => 2.5ms environ
					//printf("I2CSlave::WriteAddressed: %c%d  %d\r\n", code, sizeof(cmd), r);

					if (r == 0)
					{
						gotoLed = !gotoLed;
						//printf("      Read CMD: %d %d %d %d\r\n", cmd[0], cmd[1], cmd[2], cmd[3]);
						float2bytes_t x;
						x.bytes[0] = cmd[0];
						x.bytes[1] = cmd[1];
						x.bytes[2] = cmd[2];
						x.bytes[3] = cmd[3];
						float2bytes_t y;
						y.bytes[0] = cmd[4];
						y.bytes[1] = cmd[5];
						y.bytes[2] = cmd[6];
						y.bytes[3] = cmd[7];
						float2bytes_t t;
						t.bytes[0] = cmd[8];
						t.bytes[1] = cmd[9];
						t.bytes[2] = cmd[10];
						t.bytes[3] = cmd[11];

						// set position
						odo->resetX(x.f); //mm
						odo->resetY(y.f);
						odo->resetTheta(t.f);

#ifdef DEBUG_COM_I2C
						printf("S12 x=%f  y=%f  t=%f \r\n", odo->getXmm(), odo->getYmm(),
								odo->getTheta());
#endif
					}
					else
					{
						printf(
								"ERROR I2CSlave::WriteAddressed : IMPOSSIBLE TO READ SECOND COMMAND for P! %d\r\n",
								r);
						ErrorLed = 1;
					}
				}
				code = 0;
				nbdata = 0;
			}

			if (code == 'Q')
			{
				stopAsserv();
#ifdef DEBUG_COM_I2C
				printf("stopAsserv\r\n");
#endif
				code = 0;
				nbdata = 0;
			}
			if (code == 'A')
			{
				startAsserv();
#ifdef DEBUG_COM_I2C
				printf("startAsserv\r\n");
#endif
				code = 0;
				nbdata = 0;
			}
		}

		break;
	}

	for (unsigned int i = 0; i < sizeof(buf); i++)
		buf[i] = 0;    // Clear buffer
	for (unsigned int i = 0; i < sizeof(cmd); i++)
		cmd[i] = 0;    // Clear buffer

	wait_us(5); //permet d'être détecté par USBPC !
}
#endif


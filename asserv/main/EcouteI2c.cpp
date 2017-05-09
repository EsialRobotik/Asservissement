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

#include "../commandManager/CommandManager.h"
#include "../consignController/ConsignController.h"
#include "../odometrie/Odometrie.h"

#ifdef LCD_ACTIVATE
#include "../../C12832/C12832.h"
#endif

//Definitions globales
#ifdef COM_I2C_ACTIVATE
I2CSlave slave(p9, p10);
#endif

#ifdef COM_I2C_ACTIVATE
void ecouteI2cConfig()
{
	slave.frequency(400000); //i2c freq 400khz
	slave.address(I2C_SLAVE_ADDRESS << 1); //We shift it left because mbed takes in 8 bit addresses

}

//Name				Permissions	writeCODE		ReadResponse			WriteAction
//Whoami			R			0x00			Device ID				N/A
//Set position 		W			'S' + 12		N/A						float/float/float write position x,y,t
//Get position		WR			'p' + 13		float/float/float/byte
//init asserv		W			'I'	+ 0x00
//stop asserv		W			'!' + 0x00
//stop managers 	W			'K' + 0x00
//start managers	W			'J' + 0x00
//emergency stop	W			'h' + 0x00
//reset emerg stop 	W			'r' + 0x00
//aVance			WR			'v' + 4
//Tourne			WR			't' + 4
//faire face		WR			'f' + 8
//Goto 				WR			'g' + 8
//Enchainement		WR			'e' + 8


void ecouteI2c(ConsignController *consignC, CommandManager *commandM, MotorsController *motorsC,
		Odometrie *odo, bool *prun)
{
	bool run = *prun;


	char buf[2]; //I2C_Packet and PACKET_SIZE
	char cmd[12];
	char cmd13[13];
	char cmd8[8];
	char cmd4[4];

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

		if (code == 'p')
		{
			if (nbdata != sizeof(cmd13))
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
				cmd13[0] = x.bytes[0];
				cmd13[1] = x.bytes[1];
				cmd13[2] = x.bytes[2];
				cmd13[3] = x.bytes[3];
				//printf("      Write : %d %d %d %d\r\n", cmd13[0], cmd13[1], cmd13[2], cmd13[3]);
				float2bytes_t y;
				y.f = odo->getYmm();
				cmd13[4] = y.bytes[0];
				cmd13[5] = y.bytes[1];
				cmd13[6] = y.bytes[2];
				cmd13[7] = y.bytes[3];
				//printf("      Write : %d %d %d %d\r\n", cmd13[4], cmd13[5], cmd13[6], cmd13[7]);
				float2bytes_t t;
				t.f = odo->getTheta();
				cmd13[8] = t.bytes[0];
				cmd13[9] = t.bytes[1];
				cmd13[10] = t.bytes[2];
				cmd13[11] = t.bytes[3];
				//printf("      Write : %d %d %d %d\r\n", cmd13[8], cmd13[9], cmd13[10], cmd13[11]);
				cmd13[12] = commandM->getLastCommandStatus();
				r = slave.write(cmd13, sizeof(cmd13));
				if (r == 0)
				{
#ifdef DEBUG_COM_I2C
					printf("p13 x=%f  y=%f  t=%f  s=%d\r\n", x.f, y.f, t.f,
							commandM->getLastCommandStatus());
#endif
#ifdef LCD_ACTIVATE
					lcd.cls();
					lcd.locate(0, 0);
					lcd.printf("p13 x%.1f y%.1f t%.1f", x.f, y.f, t.f);
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
				printf("I2CSlave::WriteAddressed: %c%d\r\n", buf[0], (int) buf[1]);
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

			//cas des commandes sans parametre

			if (code == 'I') 		//------ I / Active les actions dans la boucle d'asservissement
			{
				initAsserv(prun);
				run = *prun;
#ifdef DEBUG_COM_I2C
				printf("%c -- initAsserv\r\n", code);
#endif
#ifdef LCD_ACTIVATE
				lcd.cls();
				lcd.locate(0, 0);
				lcd.printf("I0 INIT");
#endif
				code = 0;
				nbdata = 0;
			}
			else if (code == '!') // !  Stoppes actions dans la boucle d'asservissement et supprime les objets
			{
				stopAsserv(prun);
				run = *prun;
#ifdef DEBUG_COM_I2C
				printf("%c -- stopAsserv\r\n", code);
#endif
#ifdef LCD_ACTIVATE
				lcd.cls();
				lcd.locate(0, 0);
				lcd.printf("!0 STOP ASSERV");
#endif
				code = 0;
				nbdata = 0;
			}
			else if (code == 'K')  	// K / desactive le consignController et le commandManager
			{
				if (run)
				{
					//uniquement odométrie active
					consignC->perform_On(false);
					commandM->perform_On(false);
#ifdef DEBUG_COM_I2C
					printf("%c -- stop consignC & commandM\r\n", code);
#endif
#ifdef LCD_ACTIVATE
					lcd.cls();
					lcd.locate(0, 0);
					lcd.printf("K0 STOP MANAGERS");
#endif
				}
				code = 0;
				nbdata = 0;
			}
			else if (code == 'J') 	// J / reactive le consignController et le commandManager
			{
				if (run)
				{
					//uniquement odométrie active
					consignC->perform_On(true);
					commandM->perform_On(true);
#ifdef DEBUG_COM_I2C
					printf("%c -- activate consignC & commandM\r\n", code);
#endif
#ifdef LCD_ACTIVATE
					lcd.cls();
					lcd.locate(0, 0);
					lcd.printf("J0 ACTIV MANAGERS");
#endif
				}
				code = 0;
				nbdata = 0;
			}
			else if (code == 'h') // h / Halte ! / Arrêt d'urgence ! Le robot est ensuite systématiquement asservi à sa position actuelle.
								  //Cela devrait suffire à arrêter le robot correctement. La seule commande acceptée par la suite sera un Reset de l'arrêt d'urgence : toute autre commande sera ignorée.
			{
				if (run)
				{
					commandM->setEmergencyStop();
#ifdef DEBUG_COM_I2C
					printf("%c -- EmergencyStop\r\n", code);
#endif
#ifdef LCD_ACTIVATE
					lcd.cls();
					lcd.locate(0, 0);
					lcd.printf("h0 EmergencyStop");
#endif
				}
				code = 0;
				nbdata = 0;
			}
			else if (code == 'r') // r / Reset de l'arrêt d'urgence / Remet le robot dans son fonctionnement normal après un arrêt d'urgence.
								  //	Les commandes en cours au moment de l'arrêt d'urgence NE sont PAS reprises. Si le robot n'est pas en arrêt d'urgence, cette commande n'a aucun effet.

			{
				if (run)
				{
					commandM->resetEmergencyStop();
#ifdef DEBUG_COM_I2C
					printf("%c -- resetEmergencyStop\r\n", code);
#endif
#ifdef LCD_ACTIVATE
					lcd.cls();
					lcd.locate(0, 0);
					lcd.printf("h0 EmergencyReset");
#endif
				}
				code = 0;
				nbdata = 0;
			}
		}
		else //cas des parametres uniquement s'il y en a
		{
			if (code == 'S') 		// S / set Position / applique la nouvelle position du robot
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
#ifdef LCD_ACTIVATE
						lcd.cls();
						lcd.locate(0, 0);
						lcd.printf("S12 x%.1f y%.1f t%.1f\n", odo->getXmm(), odo->getYmm(),
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
			else if (code == 'v') // v%d\n / aVancer / d : entier, en mm / Fait avancer le robot de d mm, tout droit
			{
				if (nbdata != sizeof(cmd4))
				{
					printf(
							"ERROR I2CSlave::WriteAddressed (code=%c) : nbdata %d != sizeof(cmd) %d !\r\n",
							code, nbdata, sizeof(cmd4));
					ErrorLed = 1;
				}
				else
				{
					r = slave.read(cmd4, sizeof(cmd4));
					//printf("I2CSlave::WriteAddressed: %c%d  %d\r\n", code, sizeof(cmd), r);

					if (r == 0)
					{
						if (run)
						{
							gotoLed = !gotoLed;
							//printf("      Read CMD: %d %d %d %d\r\n", cmd[0], cmd[1], cmd[2], cmd[3]);
							float2bytes_t mm;
							mm.bytes[0] = cmd4[0];
							mm.bytes[1] = cmd4[1];
							mm.bytes[2] = cmd4[2];
							mm.bytes[3] = cmd4[3];

							commandM->addStraightLine(mm.f);
#ifdef DEBUG_COM_I2C
							printf("v4 dist=%f \r\n", mm.f);
#endif
#ifdef LCD_ACTIVATE
							lcd.cls();
							lcd.locate(0, 0);
							lcd.printf("v4 dist=%.1f\n", mm.f);
#endif
						}
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
			else if (code == 't') // t%a\n / Tourner / a : entier, en degrées / Fait tourner le robot de a degrées. Le robot tournera dans le sens trigonométrique : si a est positif, il tourne à gauche, et vice-versa.
			{
				if (nbdata != sizeof(cmd4))
				{
					printf("ERROR I2CSlave::WriteAddressed (code=%c) : nbdata != sizeof(cmd) !\r\n",
							code);
					ErrorLed = 1;
				}
				else
				{
					r = slave.read(cmd4, sizeof(cmd4));
					//printf("I2CSlave::WriteAddressed: %c%d  %d\r\n", code, sizeof(cmd), r);

					if (r == 0)
					{
						if (run)
						{
							gotoLed = !gotoLed;
							//printf("      Read CMD: %d %d %d %d\r\n", cmd[0], cmd[1], cmd[2], cmd[3]);
							float2bytes_t deg;
							deg.bytes[0] = cmd4[0];
							deg.bytes[1] = cmd4[1];
							deg.bytes[2] = cmd4[2];
							deg.bytes[3] = cmd4[3];

							commandM->addTurn(deg.f);
#ifdef DEBUG_COM_I2C
							printf("t4 degrees=%f \r\n", deg.f);
#endif
#ifdef LCD_ACTIVATE
							lcd.cls();
							lcd.locate(0, 0);
							lcd.printf("t4 deg=%0.1f\n", deg.f);
#endif
						}
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
			else if (code == 'f') // f%x%y / faire Face / x, y : entiers, en mm / Fait tourner le robot pour être en face du point de coordonnées (x, y). En gros, ça réalise la première partie d'un Goto : on se tourne vers le point cible, mais on avance pas.
			{
				if (nbdata != sizeof(cmd8))
				{
					printf("ERROR I2CSlave::WriteAddressed (code=%c) : nbdata != sizeof(cmd) !\r\n",
							code);
					ErrorLed = 1;
				}
				else
				{
					r = slave.read(cmd8, sizeof(cmd8));
					//printf("I2CSlave::WriteAddressed: %c%d  %d\r\n", code, sizeof(cmd), r);

					if (r == 0)
					{
						if (run)
						{
							gotoLed = !gotoLed;
							//printf("      Read CMD: %d %d %d %d\r\n", cmd[0], cmd[1], cmd[2], cmd[3]);
							float2bytes_t x;
							x.bytes[0] = cmd8[0];
							x.bytes[1] = cmd8[1];
							x.bytes[2] = cmd8[2];
							x.bytes[3] = cmd8[3];
							float2bytes_t y;
							y.bytes[0] = cmd8[4];
							y.bytes[1] = cmd8[5];
							y.bytes[2] = cmd8[6];
							y.bytes[3] = cmd8[7];

							commandM->addGoToAngle(x.f, y.f);
#ifdef DEBUG_COM_I2C
							printf("f8 x=%f y=%f\r\n", x.f, y.f);
#endif
#ifdef LCD_ACTIVATE
							lcd.cls();
							lcd.locate(0, 0);
							lcd.printf("f8 x=%.1f y=%.1f\n", x.f, y.f);
#endif
						}
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
			else if (code == 'g') //  g%x#%y\n / Goto / x, y : entiers, en mm /Le robot se déplace au point de coordonnée (x, y). Il tourne vers le point, puis avance en ligne droite. L'angle est sans cesse corrigé pour bien viser le point voulu.
			{
				if (nbdata != sizeof(cmd8))
				{
					printf("ERROR I2CSlave::WriteAddressed (code=%c) : nbdata != sizeof(cmd) !\r\n",
							code);
					ErrorLed = 1;
				}
				else
				{
					r = slave.read(cmd8, sizeof(cmd8));
					//printf("I2CSlave::WriteAddressed: %c%d  %d\r\n", code, sizeof(cmd), r);
					if (r == 0)
					{
						if (run)
						{
							gotoLed = !gotoLed;
							//printf("      Read CMD: %d %d %d %d\r\n", cmd[0], cmd[1], cmd[2], cmd[3]);
							float2bytes_t x;
							x.bytes[0] = cmd8[0];
							x.bytes[1] = cmd8[1];
							x.bytes[2] = cmd8[2];
							x.bytes[3] = cmd8[3];
							float2bytes_t y;
							y.bytes[0] = cmd8[4];
							y.bytes[1] = cmd8[5];
							y.bytes[2] = cmd8[6];
							y.bytes[3] = cmd8[7];

							commandM->addGoTo(x.f, y.f);
#ifdef DEBUG_COM_I2C
							printf("g8 x=%f y=%f\r\n", x.f, y.f);
#endif
#ifdef LCD_ACTIVATE
							lcd.cls();
							lcd.locate(0, 0);
							lcd.printf("g8 x=%.1f y=%.1f\n", x.f, y.f);
#endif
						}
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
			else if (code == 'e') //  e%x#%y\n / goto Enchaîné / x, y : entiers, en mm / Idem que le Goto, sauf que lorsque le robot est proche du point d'arrivée (x, y), on s'autorise à enchaîner directement la consigne suivante si c'est un Goto ou un Goto enchaîné, sans marquer d'arrêt.
			{
				if (nbdata != sizeof(cmd8))
				{
					printf("ERROR I2CSlave::WriteAddressed (code=%c) : nbdata != sizeof(cmd) !\r\n",
							code);
					ErrorLed = 1;
				}
				else
				{
					r = slave.read(cmd8, sizeof(cmd8));
					//printf("I2CSlave::WriteAddressed: %c%d  %d\r\n", code, sizeof(cmd), r);
					if (r == 0)
					{
						if (run)
						{
							gotoLed = !gotoLed;
							//printf("      Read CMD: %d %d %d %d\r\n", cmd[0], cmd[1], cmd[2], cmd[3]);
							float2bytes_t x;
							x.bytes[0] = cmd8[0];
							x.bytes[1] = cmd8[1];
							x.bytes[2] = cmd8[2];
							x.bytes[3] = cmd8[3];
							float2bytes_t y;
							y.bytes[0] = cmd8[4];
							y.bytes[1] = cmd8[5];
							y.bytes[2] = cmd8[6];
							y.bytes[3] = cmd8[7];

							commandM->addGoToEnchainement(x.f, y.f);
#ifdef DEBUG_COM_I2C
							printf("e8 x=%f y=%f\r\n", x.f, y.f);
#endif
#ifdef LCD_ACTIVATE
							lcd.cls();
							lcd.locate(0, 0);
							lcd.printf("e8 x=%.1f y=%.1f\n", x.f, y.f);
#endif
						}
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


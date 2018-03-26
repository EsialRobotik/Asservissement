#ifndef MAIN
#define MAIN

#include "mbed.h"
#include "../config/config.h"
#include "../odometrie/Odometrie.h"
#include "../motorsController/Md25/Md25ctrl.h"
//#include "../motorsController/Md22/Md22.h"
//#include "../motorsController/Qik/Qik.h"
//#include "../motorsController/PololuSMCs/PololuSMCs.h"
#include "../motorsController/DummyMotorsController.h"
#include "../consignController/ConsignController.h"
#include "../commandManager/CommandManager.h"

//#define LCD_ACTIVATE 1
#define COM_SERIEPC_ACTIVATE 1

#include "../config/config.h"
#include "../odometrie/Odometrie.h"
#include "../motorsController/Md25/Md25ctrl.h"
#include "../motorsController/DummyMotorsController.h"
#include "../consignController/ConsignController.h"
#include "../commandManager/CommandManager.h"
#ifdef DEBUG_UDP
#include "../debug/DebugUDP.h"
#endif
#ifdef LCD_ACTIVATE
#include "../../C12832/C12832.h"
C12832 lcd(p5, p7, p6, p8, p11);
#endif

//#define ASSERV_PERIOD 0.002 //en s

// Ticker pour l'interruption de la boucle d'asserv
Ticker Live;
void Live_isr(void);

int leftSpeed, rightSpeed;
bool run = false;

// Méthodes utiles
int main();
void startAsserv();
void stopAsserv(bool *prun);
void initAsserv(bool *prun);
void resetAsserv();
void ecouteSeriePC();

void ecouteSerie();
void parseGoto(void);
void parseCommandeOdometrie(void);
void parseCommandeRegulateur(void);
void parseCommandeConfig(void);


// Objets qui vont bien pour asservir le bestiau
Odometrie *odometrie;
MotorsController *motorController;
ConsignController *consignController;
CommandManager *commandManager;
DigitalOut refLed(LED2); //si allumer
DigitalOut liveLed(LED4); //clignote si Live_isr est en route
DigitalOut gotoLed(LED3); //clignote a chaque nouvelle commande

DigitalOut ErrorLed(LED1);

#ifdef DEBUG_UDP
DigitalOut dataLed(LED1);
char debugLedStatus;
DebugUDP *debugUdp;
UDPSocket udp;
uint64_t temps;
#endif

#endif

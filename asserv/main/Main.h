#ifndef MAIN
#define MAIN

#include "mbed.h"
#include "../config/config.h"
#include "../odometrie/Odometrie.h"
#include "../motorsController/Md25/Md25ctrl.h"
#include "../motorsController/DummyMotorsController.h"
#include "../consignController/ConsignController.h"
#include "../commandManager/CommandManager.h"

#define LCD_ACTIVATE 1
#define COM_SERIEPC_ACTIVATE 1

#include "mbed.h"
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
#endif
#define ASSERV_PERIOD 0.002 //en s

// Ticker pour l'interruption de la boucle d'asserv
Ticker Live;
void Live_isr(void);

int leftSpeed;

// Méthodes utiles
int main();
void startAsserv();
void stopAsserv();
void initAsserv();
void resetAsserv();
void ecouteSeriePC();

void ecouteSerie();

// Objets qui vont bien pour asservir le bestiau
Odometrie *odometrie;
MotorsController *motorController;
ConsignController *consignController;
CommandManager *commandManager;
DigitalOut refLed(LED2);
DigitalOut liveLed(LED4);
DigitalOut gotoLed(LED3);
#ifdef DEBUG_UDP
DigitalOut dataLed(LED1);
char debugLedStatus;
DebugUDP *debugUdp;
UDPSocket udp;
uint64_t temps;
#endif

#endif

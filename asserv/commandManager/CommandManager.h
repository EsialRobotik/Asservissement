#ifndef COMMAND_MANAGER
#define COMMAND_MANAGER

#include "mbed.h"
#include "../consignController/ConsignController.h"
#include "CMDList/CMDList.h"
#include "../Utils/Utils.h"
#include "../config/config.h"

class CommandManager
{

public:
    CommandManager(int capacity , ConsignController *ctrlr, Odometrie *odo);
    ~CommandManager();

    bool addStraightLine(int32_t valueInmm);
    bool addTurn(int32_t angleInDeg);
    bool addGoTo(int32_t posXInmm, int32_t posYInmm);
    bool addGoToBack(int32_t posXInmm, int32_t posYInmm);
    bool addGoToEnchainement(int32_t posXInmm, int32_t posYInmm);
    bool addGoToAngle(int32_t posXInmm, int32_t posYInmm);
    void perform();

    // Gestion d'un éventuel arrêt d'urgence
    void setEmergencyStop();
    void resetEmergencyStop();

    // Statut de la dernière commande
    int getLastCommandStatus() { return lastStatus; }
    void setLastCommandStatus(int s) { lastStatus = s; }

private:
    CMDList *liste; //File d'attente des commandes
    ConsignController *cnsgCtrl;
    Odometrie *odometrie; // Odométrie, pour pouvoir faire les conversion et calculer la cible
    CMD currCMD; //commande courante
    CMD nextCMD; //commande suivante

    bool emergencyStop; //arrêt d'urgence : si le robot adverse est dans les parages

    bool currentConsignFinished; //signale si la consigne courante est terminée. Reste à vrai tant qu'une nouvelle consigne n'est pas arrivée.
    int lastStatus;

    double computeDeltaTheta(double deltaX, double deltaY); // Calcul de l'angle à parcourir
    int64_t computeDeltaDist(double deltaX, double deltaY); // Calcul de la distance à parcourir
    // GoTo là où on veut
    void computeGoTo();
    void computeGoToBack();
    void computeGoToAngle();
    void computeEnchainement(); // Tentative d'enchainement
};

#endif

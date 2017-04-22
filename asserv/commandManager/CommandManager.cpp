#include "CommandManager.h"


//--------------------------------------------------------
//                       COMMAND MANAGER
//--------------------------------------------------------


/*
 * Fonctions de "commandes"  externes, tout droit, tourner, arc de cercle, aller au point (x,y), ...
 */


CommandManager::CommandManager(int capacity , ConsignController *ctrlr, Odometrie *odo)
{
    liste = new CMDList(capacity);
    cnsgCtrl = ctrlr;
    odometrie = odo;
    currCMD.type = CMD_NULL;
    nextCMD.type = CMD_NULL;
    commandStatus = STATUS_IDLE;
}

CommandManager::~CommandManager()
{
    delete liste;
}

/*
 * Pour ajouter des commandes à la file, on donne la position à parcourir en mm ou en degré,
 * le commandManager fait les convertion en UO lui même
 */
bool CommandManager::addStraightLine(int32_t valueInmm)
{
    return liste->enqueue(CMD_GO , Utils::mmToUO(odometrie, valueInmm), 0);
}

bool CommandManager::addTurn(int32_t angleInDeg)
{
    return liste->enqueue(CMD_TURN , Utils::degToUO(odometrie, angleInDeg) , 0);
}

bool CommandManager::addGoTo(int32_t posXInmm, int32_t posYInmm)
{
    return liste->enqueue(CMD_GOTO , Utils::mmToUO(odometrie, posXInmm) , Utils::mmToUO(odometrie, posYInmm));
}

bool CommandManager::addGoToBack(int32_t posXInmm, int32_t posYInmm)
{
    return liste->enqueue(CMD_GOTO_BACK , Utils::mmToUO(odometrie, posXInmm) , Utils::mmToUO(odometrie, posYInmm));
}

bool CommandManager::addGoToEnchainement(int32_t posXInmm, int32_t posYInmm)
{
    return liste->enqueue(CMD_GOTOENCHAIN , Utils::mmToUO(odometrie, posXInmm) , Utils::mmToUO(odometrie, posYInmm));
}

bool CommandManager::addGoToAngle(int32_t posXInmm, int32_t posYInmm)
{
    return liste->enqueue(CMD_GOTOANGLE , Utils::mmToUO(odometrie, posXInmm) , Utils::mmToUO(odometrie, posYInmm));
}


/*
 * Fonction de mise a jour...
 */
void CommandManager::perform()
{
    // Arrêt d'urgence ! On n'accepte aucune commande.
    if (commandStatus == STATUS_HALTED) {
        while (currCMD.type != CMD_NULL) { //On s'assure que la liste des commandes est vide
            currCMD = liste->dequeue();
        }

        nextCMD.type = CMD_NULL;
        return;
    }

    /*
    * Si la QuadRampDerivee n'est pas terminée, dans le cas d'un GoTo on ajuste la trajectoire
    * Sinon on attend tranquillement la fin
    */
    if (!cnsgCtrl->areRampsFinished()) {
        // On est forcément en train d'exécuter une consigne
        commandStatus = STATUS_RUNNING;

        if (currCMD.type == CMD_GO || currCMD.type == CMD_TURN) {  // On avance ou on tourne sur place
            return; //Dans ce cas, on attend simplement d'etre arrive :)
        } else if (currCMD.type == CMD_GOTO) { // On est en plein GoTo, donc on est en train de se planter et on ajuste
            computeGoTo();
        } else if (currCMD.type == CMD_GOTO_BACK) {
            computeGoToBack();
        } else if (currCMD.type == CMD_GOTOANGLE) { // On est en plein GoTo en angle, donc on est en train de se planter et on ajuste
            computeGoToAngle();
        } else if (currCMD.type == CMD_GOTOENCHAIN) { // Là, on est vraiment en train de se planter parce qu'on veut enchainer les consignes
            computeEnchainement();
        }

        if (nextCMD.type == CMD_NULL) { // On a pas chargé de consigne suivante
            nextCMD = liste->dequeue(); // Du coup, on essaie...
        }

        /*
        * Si on a terminé la QuadRampDerivee, on passe à la commande suivante
        */
    } else {

        // ToDo Réfléchir à l'enchainement de commande pour ne pas s'arrêter au moment de calculer la suivante
        if (nextCMD.type != CMD_NULL) { //On a déjà chargée une consigne
            currCMD = nextCMD; // La consigne suivante devient la consigne courante
            nextCMD = liste->dequeue(); // On essaye de récupérer la prochaine consigne
        } else {
            currCMD = liste->dequeue(); // On prend la consigne suivante immédiatement
        }

        if (currCMD.type == CMD_NULL) {  // S'il n'y a plus de commande, on est arrivé à bon port
            commandStatus = STATUS_IDLE;
            return; // Il n'y en a pas...
        }

        // On a une consigne à exécuter !
        commandStatus = STATUS_RUNNING;

        if (currCMD.type == CMD_GO) {  // On avance ou on recule de la consigne
            cnsgCtrl->add_dist_consigne(currCMD.value);
        } else if (currCMD.type == CMD_TURN) {   // On tourne de la consigne
            cnsgCtrl->add_angle_consigne(currCMD.value);
        } else if (currCMD.type == CMD_GOTO) {   // On appel computeGoTo qui se débrouille pour aller en (x,y)
            computeGoTo();
        } else if (currCMD.type == CMD_GOTO_BACK) {
            computeGoToBack();
        } else if (currCMD.type == CMD_GOTOENCHAIN) {
            computeEnchainement(); //On va tenter d'enchainer la consigne suivante
        } else if (currCMD.type == CMD_GOTOANGLE) { // On appel computeGoToAngle qui se débrouille pour s'aligner avec (x,y)
            computeGoToAngle();
        }
    }
}

/*
 * On a une commande GoTo(x,y) avec x et y deux points dans le repère du robot
 * (0,0) est la position initiale du robot après calage bordure
 * TODO décider de cette connerie
 */
void CommandManager::computeGoTo()
{

    double deltaX = currCMD.value - odometrie->getX(); // Différence entre la cible et le robot selon X
    double deltaY = currCMD.secValue - odometrie->getY();  // Différence entre la cible et le robot selon Y

    // Valeur absolue de la distance à parcourir en allant tout droit pour atteindre la consigne
    int64_t deltaDist = computeDeltaDist(deltaX, deltaY);

    // La différence entre le thetaCible (= cap à atteindre) et le theta (= cap actuel du robot) donne l'angle à parcourir
    double deltaTheta = computeDeltaTheta(deltaX, deltaY);

    /*//TODO à tester en conditions réelles et extrêmes de mauvaises utilisations
    if ( fabs(deltaTheta) < Config::angleThreshold ) { // Si on est dans la fenêtre de départ en angle
      // La consigne est la somme de la distance à parcourir et de notre position dans l'accu
      int64_t consigne_dist = deltaDist + cnsgCtrl->getAccuDist();
      cnsgCtrl->set_dist_consigne( consigne_dist ); // On set la consigne
    }
    // Dans tout les cas, on essaie d'atteindre le bon cap
    // La consigne à atteindre en angle est la somme du deltaTheta en UO et de l'accumulateur du régu
    int64_t consigne_angle = Utils::radToUO(odometrie, deltaTheta) + cnsgCtrl->getAccuAngle();
    cnsgCtrl->set_angle_consigne( consigne_angle ); // On set la consigne*/

    //  Ancienne version, juste au cas où. A virer une fois operationnelle
    // On projette la distance à parcourir sur l'axe X du repaire mobile du robot
    double projectedDist = deltaDist * cos(deltaTheta);
    //printf("dd=%lld - rT=%lld - rTUO=%lld - ", deltaDist, Config::returnThreshold, Utils::mmToUO(odometrie, Config::returnThreshold));
    int64_t consigne_dist;

    if (deltaDist < Utils::mmToUO(odometrie, Config::returnThreshold)) {
        consigne_dist = projectedDist + cnsgCtrl->getAccuDist();
        cnsgCtrl->set_dist_consigne(consigne_dist);
    } else {
        cnsgCtrl->set_angle_consigne(
            Utils::radToUO(odometrie, deltaTheta) + cnsgCtrl->getAccuAngle()
        ) ; //on se met dans la bonne direction

        //printf("dT=%ldd - ",Utils::radToUO(odometrie, deltaTheta) + cnsgCtrl->getAccuAngle());
        //printf("Td=%f - aT=%f\n", fabs( thetaCible - odometrie->getTheta()), Config::angleThreshold);
        if (fabs(deltaTheta) < Config::angleThreshold) {
            consigne_dist = deltaDist + cnsgCtrl->getAccuDist();
        } else {
            consigne_dist = cnsgCtrl->getAccuDist();
        }

        cnsgCtrl->set_dist_consigne(consigne_dist);
        //printf("Cd=%lld\n", consigne_dist);
    }

}

/*
 * On a une commande GoToBack(x,y) avec x et y deux points dans le repère du robot
 */
void CommandManager::computeGoToBack()
{

    double deltaX = currCMD.value - odometrie->getX(); // Différence entre la cible et le robot selon X
    double deltaY = currCMD.secValue - odometrie->getY();  // Différence entre la cible et le robot selon Y

    // Valeur absolue de la distance à parcourir en allant tout droit pour atteindre la consigne
    int64_t deltaDist = computeDeltaDist(deltaX, deltaY);

    // La différence entre le thetaCible (= cap à atteindre) et le theta (= cap actuel du robot) donne l'angle à parcourir
    // Comme on veux aller en marche arrière (computeGoToBack), on utilise
    // -deltaX et -deltaY pour calculer l'angle.
    double deltaTheta = computeDeltaTheta(-deltaX, -deltaY);

    // On projette la distance à parcourir sur l'axe X du repaire mobile du robot
    double projectedDist = deltaDist * cos(deltaTheta);
    //printf("dd=%lld - rT=%lld - rTUO=%lld - ", deltaDist, Config::returnThreshold, Utils::mmToUO(odometrie, Config::returnThreshold));
    int64_t consigne_dist;

    // On utilise -projectedDist et -deltaDist parc qu'on veut reculer
    if (deltaDist < Utils::mmToUO(odometrie, Config::returnThreshold)) {
        consigne_dist =  cnsgCtrl->getAccuDist() - projectedDist;
        cnsgCtrl->set_dist_consigne(consigne_dist);
    } else {
        cnsgCtrl->set_angle_consigne(
            Utils::radToUO(odometrie, deltaTheta) + cnsgCtrl->getAccuAngle()
        ) ; //on se met dans la bonne direction

        //printf("dT=%ldd - ",Utils::radToUO(odometrie, deltaTheta) + cnsgCtrl->getAccuAngle());
        //printf("Td=%f - aT=%f\n", fabs( thetaCible - odometrie->getTheta()), Config::angleThreshold);
        if (fabs(deltaTheta) < Config::angleThreshold) {
            consigne_dist = cnsgCtrl->getAccuDist() - deltaDist;
        } else {
            consigne_dist = cnsgCtrl->getAccuDist();
        }

        cnsgCtrl->set_dist_consigne(consigne_dist);
        //printf("Cd=%lld\n", consigne_dist);
    }

}

/*
 * On a une commande GoToAngle(x,y), on veut que le cap du robot pointe vers ce point
 */
void CommandManager::computeGoToAngle()
{

    double deltaX = currCMD.value - odometrie->getX(); // Différence entre la cible et le robot selon X
    double deltaY = currCMD.secValue - odometrie->getY();  // Différence entre la cible et le robot selon Y

    // Angle à parcourir
    double deltaTheta = computeDeltaTheta(deltaX, deltaY);

    //TODO a tester en conditions réelles et extrêmes de mauvaises utilisations
    // La consigne à atteindre en angle est la somme du deltaTheta en UO et de l'accumulateur du régu
    int64_t consigne_angle = Utils::radToUO(odometrie, deltaTheta) + cnsgCtrl->getAccuAngle();
    cnsgCtrl->set_angle_consigne(consigne_angle);   // On set la consigne

}

/*
 * Calcul de l'angle à parcourir par le robot, ça sert souvent...
 */
double CommandManager::computeDeltaTheta(double deltaX, double deltaY)
{

    // Cap que doit atteindre le robot
    double thetaCible = atan2(deltaY, deltaX);

    // La différence entre le thetaCible (= cap à atteindre) et le theta (= cap actuel du robot) donne l'angle à parcourir
    double deltaTheta = thetaCible - odometrie->getTheta();

    // On ajuste l'angle à parcourir pour ne pas faire plus d'un demi-tour
    // Exemple, tourner de 340 degrés est plus chiant que de tourner de -20 degrés
    if (deltaTheta > PI) {
        deltaTheta -= 2.0 * PI;
    } else if (deltaTheta < -PI) {
        deltaTheta += 2.0 * PI;
    }

    return deltaTheta;
}

int64_t CommandManager::computeDeltaDist(double deltaX, double deltaY)
{
    // On a besoin de min et max pour le calcul de la racine carrée
    double max = fabs(deltaX) > fabs(deltaY) ? fabs(deltaX) : fabs(deltaY);
    double min = fabs(deltaX) <= fabs(deltaY) ? fabs(deltaX) : fabs(deltaY);

    // Valeur absolue de la distance à parcourir en allant tout droit pour atteindre la consigne
    if (max != 0) {
        return (int64_t)(max * sqrt(1.0 + (min / max) * (min / max)));
    } else {
        return 0;
    }
}

void CommandManager::computeEnchainement()
{

    // Ok, on est dans un Goto, alors, on calcule le Goto.
    computeGoTo();

    // Si la consigne suivante n'est pas un GOTO, on ne fait rien.
    if (nextCMD.type != CMD_GOTO && nextCMD.type != CMD_GOTOENCHAIN) {
        return;
    }

    //Bon, maintenant, on va checker notre distance par rapport à la consigne
    double deltaX = currCMD.value - odometrie->getX(); // Différence entre la cible et le robot selon X
    double deltaY = currCMD.secValue - odometrie->getY();  // Différence entre la cible et le robot selon Y
    int64_t deltaDist = computeDeltaDist(deltaX, deltaY);

    if (deltaDist < Config::enchainThreshold) { // On a le droit de passer à la consigne suivante
        currCMD = nextCMD; // La consigne suivante devient la consigne courante
        nextCMD = liste->dequeue(); // On essaye de récupérer la prochaine consigne

        // On vient de terminer la consigne courante, on le signale en haut lieu
        putchar('d');
        putchar('\r');
        putchar('\n');

        // Le reste, c'est pas grave, on le calculera à la prochaine itération
    }

}

void CommandManager::setEmergencyStop()   //Gestion d'un éventuel arrêt d'urgence
{
    cnsgCtrl->set_dist_consigne(cnsgCtrl->getAccuDist());
    cnsgCtrl->set_angle_consigne(cnsgCtrl->getAccuAngle());

    while (currCMD.type != CMD_NULL) {
        currCMD = liste->dequeue();
    }

    commandStatus = STATUS_HALTED;
}

void CommandManager::resetEmergencyStop()
{
    if(commandStatus == STATUS_HALTED) {
        commandStatus = STATUS_IDLE;
    }
}

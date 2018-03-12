#include "Main.h"
#include <iostream>
#include <cinttypes>

extern serial_t stdio_uart;

extern "C" void HardFault_Handler()
{
    error("Planté !!\r\n");
}

void loadConfig() {
    Config::loadFile("/local/config.txt");
    printf("Version configuration : %lld\r\n", Config::configVersion);
}

Serial pc(USBTX, USBRX);

int main()
{
    // Initialisation du port série sur USB (utilisé par printf & co)
    pc.baud(115200);

    printf("--- Asservissement Nancyborg ---\r\n");
    printf("Version " GIT_VERSION " - Compilée le " DATE_COMPIL " par " AUTEUR_COMPIL "\r\n\r\n");

    LocalFileSystem local("local");

    loadConfig();
    initAsserv();

    refLed = 1;
    gotoLed = 0;
#ifdef DEBUG
    debugUdp = new DebugUDP(commandManager, odometrie);
    dataLed = 0;
    receiveLed = 0;
    liveLed = 0;
#endif

    // On attache l'interruption timer à la méthode Live_isr
    Live.attach(Live_isr, 0.005);

    //On est prêt !
    //printf("GOGO !");

    while (1) {
        ecouteSerie();
    }
}

void ecouteSerie()
{
    int32_t consigneValue;
    char c = getchar();

    switch (c) {
        // Commandes basiques
        case 'h': //Arrêt d'urgence
            commandManager->setEmergencyStop();
            break;

        case 'r': //Reset de l'arrêt d'urgence
            commandManager->resetEmergencyStop();
            break;

        case 'v': //aVance d'un certain nombre de mm
            scanf("%" PRIi32, &consigneValue);
            commandManager->addStraightLine(consigneValue);
            break;

        case 't': //Tourne d'un certain angle en degrés
            scanf("%" PRIi32, &consigneValue);
            commandManager->addTurn(consigneValue);
            break;
        // Fin commandes basiques

        // Commandes GOTO.
        case 'g':
            parseGoto();
            break;

        // Commande de feedback
        case 'p': //retourne la Position et l'angle courants du robot
            printf("x%" PRIi32 "y%" PRIi32 "a%lf\r\n",
                    (int32_t) Utils::UOTomm(odometrie, odometrie->getX()),
                    (int32_t) Utils::UOTomm(odometrie, odometrie->getY()),
                    odometrie->getTheta());
            break;

        // Commandes de l'odométrie
        case 'O':
            parseCommandeOdometrie();
            break;

        // Commandes des réglateurs
        case 'R':
            parseCommandeRegulateur();
            break;

        // Commandes de configuration
        case 'C':
            parseCommandeConfig();
            break;

        default:
            //putchar(c);
            break;
    }
}

void initAsserv() {
    printf("Creation des objets... ");
    fflush(stdout);

    odometrie = new Odometrie();
    motorController = new Md22(p9, p10);
    //motorController = new Qik(p9, p10);
    //motorController = new PololuSMCs(p13, p14, p28, p27);

    consignController = new ConsignController(odometrie, motorController);
    commandManager = new CommandManager(50, consignController, odometrie);

    printf("ok\r\n");
}

void resetAsserv()
{
    printf("Réinitialisation de l'asserv...\r\n");
    //On arrête le traitement de l'asserv
    Live.detach();

    // On détruit tout les objets (sauf les moteurs, on s'en fiche de ça)
    delete odometrie;
    odometrie = NULL;

    delete consignController;
    consignController = NULL;

    delete commandManager;
    commandManager = NULL;

    delete motorController;
    motorController = NULL;

    // Et on les refait !!!
    initAsserv();

#ifdef DEBUG
    debugUdp->setNewObjectPointers(commandManager, odometrie);
#endif

    //On reprend l'asserv
    Live.attach(Live_isr, 0.005);
}

// On rafraichit l'asservissement régulièrement
void Live_isr()
{
    static int mod = 0;
    odometrie->refresh();

    if (!Config::disableAsserv) {
        consignController->perform();
        commandManager->perform();
    }

    liveLed = 1 - liveLed;


    if ((mod++) % 20 == 0) {
        printf("#%" PRIi32 ";%" PRIi32 ";%lf;%" PRIi32 ";%" PRIi32 ";%" PRIi32 ";%" PRIi32 "\r\n",
                (int32_t)Utils::UOTomm(odometrie, odometrie->getX()),
                (int32_t)Utils::UOTomm(odometrie, odometrie->getY()),
                (double) odometrie->getTheta(),
                (int32_t) commandManager->getCommandStatus(),
                (int32_t) commandManager->getPendingCommandCount(),
                (int32_t) motorController->getVitesseG() * (Config::inverseMoteurG ? -1 : 1),
                (int32_t) motorController->getVitesseD() * (Config::inverseMoteurD ? -1 : 1));
    }
#ifdef DEBUG
    temps++;
    static int refreshPeriod = 0;

    if (refreshPeriod++ == 10) {
        Net::poll();

        if (debugUdp->getDebugSend()) {
            debugUdp->addData("t", (double)(temps * 0.05));
            debugUdp->sendData();
            dataLed = 1 - dataLed;
        }

        refreshPeriod = 0;
    } else {
        dataLed = 0;
        debugUdp->dropCurrentData();
    }
#endif
}

void parseGoto(void) {
    int32_t consigneX = 0;
    int32_t consigneY = 0;
    char c = getchar(); // On récupère le type de Goto

    // Chaque Goto prend deux paramètres : X et Y
    scanf("%" PRIi32 "#%" PRIi32, &consigneX, &consigneY); //X, Y
    gotoLed = !gotoLed;

    switch(c) {
        case 'o': // Goto normal
            commandManager->addGoTo(consigneX, consigneY);
            break;

        case 'e': // Goto enchainé
            commandManager->addGoToEnchainement(consigneX, consigneY);
            break;

        case 'f': // Goto angle (faire face)
            commandManager->addGoToAngle(consigneX, consigneY);
            break;

        case 'b': // Goto en marche arrière (backward)
            commandManager->addGoToBack(consigneX, consigneY);
            break;
    }
}

void parseCommandeOdometrie(void) {
    char c1, c2;
    double consigneValue;

    // On récupère le type de commande
    c1 = getchar();
    c2 = getchar();

    // On attend une valeur en consigne
    scanf("%lf", &consigneValue);

    // Les commandes d'odométrie n'ont que des 'set'
    if(c1 == 's')
    {
        switch(c2) {
            case 'a': // Définition de l'angle
                odometrie->setTheta(consigneValue);
                break;

            case 'x': // Définition de la position en X
                odometrie->setX(Utils::mmToUO(odometrie, (int32_t) consigneValue));
                break;

            case 'y': // Définition de la position en Y
                odometrie->setY(Utils::mmToUO(odometrie, (int32_t) consigneValue));
                break;
        }
    }
}

void parseCommandeRegulateur(void) {
    char c1, c2;

    // On récupère le type de commande
    c1 = getchar();
    c2 = getchar();

    switch(c1) {
        case 'l': // Réglage de la vitesse lente
            switch(c2) {
                case 'd': // Désactivation
                    consignController->setLowSpeed(false);
                    break;

                case 'e': // Activation
                    consignController->setLowSpeed(true);
                    break;
            }
            break;

        case 'a': // Paramétrage du régu d'angle
            switch(c2) {
                case 'd': // Désactivation
                    consignController->angle_Regu_On(false);
                    break;

                case 'e': // Activation
                    consignController->angle_Regu_On(true);
                    break;

                case 'r': // Reset
                    consignController->reset_regu_angle();
                    break;
            }
            break;

        case 'd': // Paramétrage du régu de distance
            switch(c2) {
                case 'd': // Désactivation
                    consignController->dist_Regu_On(false);
                    break;

                case 'e': // Activation
                    consignController->dist_Regu_On(true);
                    break;

                case 'r': // Reset
                    consignController->reset_regu_dist();
                    break;
            }
            break;
    }
}

void parseCommandeConfig(void) {
    std::string name, value;
    const Parameter *param;
    char c = getchar(); // On récupère le type de commande

    switch(c) {
        case 'R': // réinitialiser l'asserv
            resetAsserv();
            break;

        case 'D': // dump la config du robot
            std::cout << Config::dumpConfig() << std::endl;
            break;

        case 'G': // lire la valeur d'un paramètre
            std::getline(std::cin, name, '\r');
            param = Config::getParam(name);

            if (param == NULL)
                std::cout << "error" << endl;
            else
                std::cout << param->toString() << std::endl;
            break;

        case 'S': // modifie la valeur d'un paramètre
            std::getline(std::cin, name, '\r');
            std::getline(std::cin, value, '\r');
            param = Config::getParam(name);

            if (param == NULL) {
                std::cout << "error" << endl;
            } else {
                param->setFromString(value);
                std::cout << "ok" << std::endl;
            }
            break;

        case 'L': // recharge la config
            loadConfig();
            std::cout << "ok" << endl;
            break;

        case 'W': // sauvegarde la config courante
            // config~1.txt = config.default.txt
            Config::saveToFile("/local/config~1.txt", "/local/config.txt");
            std::cout << "ok" << endl;
            break;

    }

}

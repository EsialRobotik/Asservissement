#include "Main.h"

#include <DigitalOut.h>
#include <LocalFileSystem.h>
#include <mbed_error.h>
#include <PinNames.h>
#include <sys/_stdint.h>
#include <Serial.h>
#include <Ticker.h>
#include <wait_api.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <cinttypes>

#include "../config/parameter.h"
#include "../Utils/Utils.h"

#if CONFIG_COM_I2C_ACTIVATE
#include "EcouteI2c.h"
#endif

extern "C" void HardFault_Handler()
{
    ErrorLed = 1;
    error("Planté !!\r\n");

}

void loadConfig()
{
    Config::loadFile("/local/config.txt");
    printf("Version configuration : %ld\r\n", Config::configVersion);
}

Serial pc(USBTX, USBRX);

int main()
{
    ErrorLed = 0;

    // Initialisation du port série sur USB (utilisé par printf & co)
    pc.baud(115200);

    printf("--- Asservissement ---\r\n");
    printf("Version " GIT_VERSION " - Compilée le " DATE_COMPIL " par " AUTEUR_COMPIL "\r\n\r\n");

    LocalFileSystem local("local");
    loadConfig();

    //on configure et démarre l'interruption timer (qui ne fait rien par defaut)
    startAsserv();

    // Et on reinitialise les objets
    //initAsserv();

    gotoLed = 0;

    //On est prêt !
    for (int n = 0; n < 10; n++) {
        wait_ms(50);
        refLed = 1;
        wait_ms(50);
        refLed = 0;
    }

    leftSpeed = 0;
    rightSpeed = 0;

#if CONFIG_COM_I2C_ACTIVATE
    ecouteI2cConfig();
#endif

#if CONFIG_LCD_ACTIVATE

    lcd.cls();
    //lcd.invert(0);
    //lcd.set_contrast(50);
    lcd.locate(0, 0);

    lcd.printf("mbed Cho v3! c=%d", lcd.get_contrast());
#endif

    while (1) {
#if CONFIG_COM_SERIE_ACTIVATE
        ecouteSerie();
#endif

#if CONFIG_COM_SERIEPC_ACTIVATE
        ecouteSeriePC();
#endif

#if CONFIG_COM_I2C_ACTIVATE
        ecouteI2c(consignController, commandManager, motorController, odometrie, &run);
#endif

        wait_us(5); //permet une meilleure detection par l'USBPC
    }
}

void startAsserv()
{
    printf("start asserv ...\r\n");
    run = false; //on lance le thread qui ne fait rien
    if (ErrorLed == 0) //s'il n'y pas d'erreur d'init.
            {
        // On attache l'interruption timer à la méthode Live_isr
        double period = Config::asservPeriod;
        if (period > 0 && period < 0.5) {

            Live.attach(Live_isr, period);
            printf("Periode asserv ok : %lf sec\r\n", period);
        } else {
            printf("pb avec la valeur de periode de l'asserv dans la config : %lf!!\r\n", period);
            ErrorLed = 1;
        }
    }
}

void ecouteSeriePC()
{

    /*
     *
     *
     * Commande / Caractères à envoyer sur la série / Paramètres / Effets obtenus

     g%x#%y\n / Goto / x, y : entiers, en mm /Le robot se déplace au point de coordonnée (x, y). Il tourne vers le point, puis avance en ligne droite. L'angle est sans cesse corrigé pour bien viser le point voulu.
     e%x#%y\n / goto Enchaîné / x, y : entiers, en mm / Idem que le Goto, sauf que lorsque le robot est proche du point d'arrivée (x, y), on s'autorise à enchaîner directement la consigne suivante si c'est un Goto ou un Goto enchaîné, sans marquer d'arrêt.
     v%d\n / aVancer / d : entier, en mm / Fait avancer le robot de d mm, tout droit
     t%a\n / Tourner / a : entier, en degrées / Fait tourner le robot de a degrées. Le robot tournera dans le sens trigonométrique : si a est positif, il tourne à gauche, et vice-versa.
     f%x#%y\n / faire Face / x, y : entiers, en mm / Fait tourner le robot pour être en face du point de coordonnées (x, y). En gros, ça réalise la première partie d'un Goto : on se tourne vers le point cible, mais on avance pas.
     h / Halte ! / Arrêt d'urgence ! Le robot est ensuite systématiquement asservi à sa position actuelle. Cela devrait suffire à arrêter le robot correctement. La seule commande acceptée par la suite sera un Reset de l'arrêt d'urgence : toute autre commande sera ignorée.
     r / Reset de l'arrêt d'urgence / Remet le robot dans son fonctionnement normal après un arrêt d'urgence. Les commandes en cours au moment de l'arrêt d'urgence NE sont PAS reprises. Si le robot n'est pas en arrêt d'urgence, cette commande n'a aucun effet.
     --c%s%r / Calage bordure / s : sens du calage bordure, r : robot ('g' : gros ; 'p' : petit) / Effectue un calage bordure. Le robot doit être dans sa zone de départ au début du calage, dirigé vers la case de départ adverse en face de la table. Il doit être assez proche de la bordure derrière lui, et pas trop proche de la bordure sur le côté. A la fin du calage, le robot est prêt à partir pour un match dans sa case de départ.
     Le choix du robot est possible, si on veut que deux robots asservis concourent en même temps sur la même table, pour qu'ils puissent faire un calage bordure en même temps sans se rentrer dedans.

     p / get Position / Récupère la position et le cap du robot sur la connexion i2c, sous la forme de 3 types float (3 * 4 bytes), avec x, y, et a les coordonnées et l'angle du robot.
     S / set Position / applique la nouvelle position du robot

     z / avance de 20 cm
     s / recule de 20 cm
     q / tourne de 45° (gauche)
     d / tourne de -45° (droite)

     M / modifie la valeur d'un paramètre / name, value
     R / réinitialiser l'asserv
     D / dump la config du robot
     G / lire la valeur d'un paramètre / name
     L / recharge la config config.txt
     W / sauvegarde la config courante  config~1.txt = config.default.txt

     I / Active les actions dans la boucle d'asservissement (odo + managers)
     ! / Stoppe actions dans la boucle d'asservissement
     K / desactive le consignController et le commandManager
     J / reactive le consignController et le commandManager

     + / applique une valeur +1 sur les moteurs LEFT
     - / applique une valeur -1 sur les moteurs LEFT


     */

    double consigneValue1 = 0;
    double consigneValue2 = 0;
    std::string name, value;
    const Parameter *param;

    if (pc.readable()) {
        gotoLed = !gotoLed;

        switch (pc.getc()) {

        case 'h': //Arrêt d'urgence
            if (!run)
                break;
            commandManager->setEmergencyStop();
            pc.printf("Arrêt d'urgence ! ");
            break;

        case 'r': //Reset de l'arrêt d'urgence
            if (!run)
                break;
            commandManager->resetEmergencyStop();
            break;

        case 'z':
            if (!run || !consignController->on())
                break;
            // Go 20cm
            //printf("consigne avant : %d\n", consignController->getDistConsigne());
            consignController->add_dist_consigne(Utils::mmToUO(odometrie, 200));
            //pc.printf("consigne apres : %d\n", consignController->getDistConsigne());
            break;

        case 's':
            if (!run || !consignController->on())
                break;
            // Backward 20cm
            //printf("consigne avant : %d\n", consignController->getDistConsigne());
            consignController->add_dist_consigne(-Utils::mmToUO(odometrie, 200));
            //pc.printf("consigne apres : %d\n", consignController->getDistConsigne());
            break;

        case 'q':
            if (!run || !consignController->on())
                break;
            // Left 45
            consignController->add_angle_consigne(Utils::degToUO(odometrie, 45));
            break;

        case 'd':
            if (!run || !consignController->on())
                break;
            // Right 45
            consignController->add_angle_consigne(-Utils::degToUO(odometrie, 45));
            break;

        case 'v': //aVance d'un certain nombre de mm
            if (!run || !commandManager->on())
                break;
            pc.scanf("%lf", &consigneValue1);
            commandManager->addStraightLine(consigneValue1);
            pc.printf("v%lf\r\n", consigneValue1);
            break;

        case 't': //Tourne d'un certain angle en degrés
            if (!run || !commandManager->on())
                break;
            pc.scanf("%lf", &consigneValue1);
            commandManager->addTurn(consigneValue1);
            pc.printf("t%lf\n", consigneValue1);
            break;

        case 'f': //faire Face à un point précis, mais ne pas y aller, juste se tourner
            if (!run || !commandManager->on())
                break;
            pc.scanf("%lf#%lf", &consigneValue1, &consigneValue2); //X, Y
            commandManager->addGoToAngle(consigneValue1, consigneValue2);
            pc.printf("g%lf#%lf\n", consigneValue1, consigneValue2);
            break;

        case 'g': //Go : va à un point précis
            if (!run || !commandManager->on())
                break;
            pc.scanf("%lf#%lf", &consigneValue1, &consigneValue2); //X, Y
            commandManager->addGoTo(consigneValue1, consigneValue2);
            //printf("g%lf#%lf\n", consigneValue1, consigneValue2);
            break;

        case 'e': // goto, mais on s'autorise à Enchainer la consigne suivante sans s'arrêter
            if (!run || !commandManager->on())
                break;
            pc.scanf("%lf#%lf", &consigneValue1, &consigneValue2); //X, Y
            commandManager->addGoToEnchainement(consigneValue1, consigneValue2);
            //printf("g%lf#%lf\n", consigneValue1, consigneValue2);
            break;

        case 'p': //retourne la Position et l'angle courants du robot
            printf("x%lfy%lfa%lfs%d\r\n", (double) Utils::UOTomm(odometrie, odometrie->getX()), (double) Utils::UOTomm(odometrie, odometrie->getY()), odometrie->getTheta(),
                    commandManager->getCommandStatus());
            break;

        case 'I': // start l'asserv
            pc.printf("I start Asserv");
            initAsserv(&run);
            break;
        case '!': // stop/quit l'asserv
            pc.printf("! stop Asserv");
            stopAsserv(&run);
            break;

        case 'R': // réinitialiser l'asserv
            if (!run)
                break;
            resetAsserv();
            break;
        case 'K': //uniquement odométrie active
            if (!run)
                break;
            consignController->perform_On(false);
            commandManager->perform_On(false);
            break;
        case 'J':
            if (!run)
                break;
            consignController->perform_On(true);
            commandManager->perform_On(true);
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

        case 'M': // modifie la valeur d'un paramètre
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

        case '+':
            if (!run)
                break;
            leftSpeed++;
            consignController->perform_On(false);
            commandManager->perform_On(false);
            consignController->setLeftSpeed(leftSpeed);
            pc.printf("LEFT+%d ", leftSpeed);
            break;
        case '-':
            if (!run)
                break;
            leftSpeed--;
            consignController->perform_On(false);
            commandManager->perform_On(false);
            consignController->setLeftSpeed(leftSpeed);
            pc.printf("LEFT-%d ", leftSpeed);
            break;

        case '*':
            if (!run)
                break;
            rightSpeed++;
            consignController->perform_On(false);
            commandManager->perform_On(false);
            consignController->setRightSpeed(rightSpeed);
            pc.printf("RIGHT+%d ", rightSpeed);
            break;
        case '/':
            if (!run)
                break;
            rightSpeed--;
            consignController->perform_On(false);
            commandManager->perform_On(false);
            consignController->setRightSpeed(rightSpeed);
            pc.printf("RIGHT-%d ", rightSpeed);
            break;

        case '3':
            // do what you want for '3'
            pc.printf("--3\r\n");

            break;
        default:
            pc.printf(" - unexpected character\r\n");
            break;
        }
    }
}

void ecouteSerie() //TODO Corriger les double/float/int64
{
    int32_t consigneValue;
    char c = getchar();

    switch (c) {
    //Commandes basiques

    case 'I': // start l'asserv
        initAsserv(&run);
        break;

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
        break; // Fin commandes basiques

        // Commandes GOTO.
    case 'g':
        parseGoto();
        break;

    case 'p':
        // Commande de feedbackcase 'p': //retourne la Position et l'angle courants du robot
        printf("x%" PRIi32 "y%" PRIi32 "a%lfs%d\r\n", (int32_t)Utils::UOTomm(odometrie, odometrie->getX()),
                (int32_t)Utils::UOTomm(odometrie, odometrie->getY()),
                odometrie->getTheta(),
                commandManager->getCommandStatus());
        break;

        // Commandes de  l'odométrie
    case 'O':
        parseCommandeOdometrie();
        break;

    case 'R':
        // Commandes des réglateurscase 'R':
        parseCommandeRegulateur();
        break;

    case 'C':
        // Commandes de configurationcase 'C':
        parseCommandeConfig();
        break;

    default:
        //putchar(c);
        break;
    }
}

void initAsserv(bool *prun)
{
    *prun = false; //pour etre sûr que isr ne fait rien
    printf("Creation des objets si necessaire... \r\n");
    fflush (stdout);

    if(codeurs == NULL)
    {
#   if CONFIG_CODEUR_DIRECTS
        // Avec des codeurs branchés directement sur la Mbed
        codeurs = new CodeursDirects(Config::pinNameList[Config::pinCodeurGchA],
                                     Config::pinNameList[Config::pinCodeurGchB],
                                     Config::pinNameList[Config::pinCodeurDchA],
                                     Config::pinNameList[Config::pinCodeurDchB]);
#   elif CONFIG_CODEUR_AVR
        // Avec des codeurs branchés sur un AVR avec lequel on communique en SPI
        codeurs = new CodeursAVR(p5, p6, p7, p8);
#   else
#       error "Undefined encoder interface; check build configuration"
#   endif
    }

    if (odometrie == NULL)
        odometrie = new Odometrie(codeurs);

    if (motorController == NULL)
    {
#   if CONFIG_MOTORCTRL_MD25
        motorController = new Md25ctrl(p28, p27);
#   elif CONFIG_MOTORCTRL_MD22
        motorController = new Md22(p9, p10);
#   elif CONFIG_MOTORCTRL_QIK
        motorController = new Qik(p9, p10);
#   elif CONFIG_MOTORCTRL_POLOLU_SMCS
        motorController = new PololuSMCs(p13, p14, p28, p27);
#   else
#       error "Undefined motor controller; check build configuration"
#   endif
    }

    if (consignController == NULL)
        consignController = new ConsignController(odometrie, motorController);
    if (commandManager == NULL)
        commandManager = new CommandManager(50, consignController, odometrie);

    *prun = true;

    //printf("ok\r\n");
}

void stopAsserv(bool *prun)
{

    //On arrête le traitement de l'asserv
    *prun = false; //afin de pouvoir supprimer les objets

    leftSpeed = 0;
    rightSpeed = 0;

    // On détruit tout les objets
    delete odometrie;
    odometrie = NULL;

    delete codeurs;
    codeurs = NULL;

    delete consignController;
    consignController = NULL;

    delete commandManager;
    commandManager = NULL;

    delete motorController;
    motorController = NULL;

    liveLed = 0;
}
void resetAsserv()
{
    printf("Réinitialisation de l'asserv...\r\n");
    stopAsserv(&run);

    ErrorLed = 0;
    //On reprend l'asserv
    initAsserv(&run);
}
#if CONFIG_COM_SERIE_ACTIVATE
static int mod = 0;
#endif
static int led = 0;

// On rafraichit l'asservissement régulièrement
void Live_isr()
{
    if (run == false)
        return;

    if ((led++) % 500 == 0) {
        liveLed = 1 - liveLed;

#if CONFIG_LCD_ACTIVATE
        lcd.locate(0, 10);
        lcd.printf("x%.1f y%.1f t%.1f  \n",
                (float) Utils::UOTomm(odometrie, odometrie->getX()),
                (float) Utils::UOTomm(odometrie, odometrie->getY()),
                (float) Utils::UOToDeg(odometrie, odometrie->getTheta()));
#endif

    }

    odometrie->refresh();

    if (!Config::disableAsserv) {
        consignController->perform();
        commandManager->perform();
    }

    liveLed = 1 - liveLed;

#if CONFIG_COM_SERIE_ACTIVATE
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
#endif
}

void parseGoto(void)
{
    int32_t consigneX = 0;
    int32_t consigneY = 0;
    char c = getchar(); // On récupère le type de Goto

    // Chaque Goto prend deux paramètres : X et Y
    scanf("%" PRIi32 "#%" PRIi32, &consigneX, &consigneY);//X, Y
    gotoLed = !gotoLed;

    switch (c) {
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

void parseCommandeOdometrie(void)
{
    char c1, c2;
    double consigneValue;

    // On récupère le type de commande
    c1 = getchar();
    c2 = getchar();

    // On attend une valeur en consigne
    scanf("%lf", &consigneValue);

    // Les commandes d'odométrie n'ont que des 'set'
    if (c1 == 's') {
        switch (c2) {
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

void parseCommandeRegulateur(void)
{
    char c1, c2;

    // On récupère le type de commande
    c1 = getchar();
    c2 = getchar();

    switch (c1) {
    case 'l': // Réglage de la vitesse lente
        switch (c2) {
        case 'd': // Désactivation
            consignController->setLowSpeed(false);
            break;

        case 'e': // Activation
            consignController->setLowSpeed(true);
            break;
        }
        break;

    case 'a': // Paramétrage du régu d'angle
        switch (c2) {
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
        switch (c2) {
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

void parseCommandeConfig(void)
{
    std::string name, value;
    const Parameter *param;
    char c = getchar(); // On récupère le type de commande

    switch (c) {
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

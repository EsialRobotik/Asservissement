#include "Odometrie.h"

#include <PinNames.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "../codeurs/CodeursDirects.h"
#include "../main/Main.h"
#include <cinttypes>

#ifdef LCD_ACTIVATE
#include "../../C12832/C12832.h"
#endif

// Constructeur
/*
 * Lors de la création de l'objet, on calcul la distance entre les roues en UO et le nombre d'UO par front
 * Les infos nécessaires au calcul sont dans config.h
 */
Odometrie::Odometrie()
{

    deltaDist = 0;
    deltaTheta = 0;

    //Instanciation des codeurs
    //codeurs = new CodeursDirects(p25, p26, p15, p16); //Avec des codeurs branchés directement sur la Mbed
    //PMX TODO a mettre en fichier de config ?
    codeurs = new CodeursDirects(p22, p21, p17, p18); //Avec des codeurs branchés directement sur la Mbed version PMX

    // Initialisation des compteurs
    compteurG = 0;
    compteurD = 0;

    // Initialisation de la position
    x = 0;
    y = 0;
    theta = 0;

    // Calcul de frontParMetre et de ratioCodeur
    if (Config::frontParMetreCodeurG != Config::frontParMetreCodeurD) {
        double min, max;

        if (Config::frontParMetreCodeurG > Config::frontParMetreCodeurD) {
            min = Config::frontParMetreCodeurD;
            max = Config::frontParMetreCodeurG;
            applyRatioOnG = false;
        } else {
            min = Config::frontParMetreCodeurG;
            max = Config::frontParMetreCodeurD;
            applyRatioOnG = true;
        }

        ratioCodeurs = max / min;
        frontParMetre = max;

    } else {
        ratioCodeurs = 1;
        frontParMetre = Config::frontParMetreCodeurD;
    }

    // Calcul de la distance entre les roues en UO
    distanceRouesUO = Config::distRoues * frontParMetre * Config::uOParFront / 1000.0;

}

// Destructeur
Odometrie::~Odometrie()
{
    delete codeurs;
}

void Odometrie::setX(int64_t xval)
{
    x = xval;
}

void Odometrie::setY(int64_t yval)
{
    y = yval;
}

void Odometrie::setTheta(double thetaVal) {
    theta = thetaVal;
}

// Mise à jour de la position du robot
void Odometrie::refresh()
{
    //Récupération des comptes des codeurs
    int32_t compteurBrutG = 0, compteurBrutD = 0;
    codeurs->getCounts(&compteurBrutG, &compteurBrutD);

    if (!Config::reglageCodeurs) {
        //On transforme ces valeurs en Unites Odometrique
        compteurD = compteurBrutD * Config::uOParFront;
        compteurG = compteurBrutG * Config::uOParFront;

        // On applique le ratio pour prendre en compte la différence entre les codeurs
        if (applyRatioOnG) {
            compteurG = compteurG * ratioCodeurs;
        } else {
            compteurD = compteurD * ratioCodeurs;
        }

        /*
         * deltaDist = la distance parcourue par le robot pendant l'itération = moyenne des distances des codeurs
         * deltaTheta = la variation de l'angle pendant l'itération = rapport de la différence des distances codeurs sur la
         *               distance entre les roues
         */
        deltaDist = (compteurG + compteurD) / 2; // En UO
        int64_t diffCount = compteurD - compteurG; // On conserve la différence entre les comptes en UO
        deltaTheta = (double) diffCount / (double) distanceRouesUO; // En radian

        if (labs(diffCount) < 1) {   // On considère le mouvement comme une ligne droite
                                     // Mise à jour de la position
            x += deltaDist * cos(theta);
            y += deltaDist * sin(theta);
        } else { //On approxime en considérant que le robot suit un arc de cercle
                 // On calcule le rayon de courbure du cercle
            double R = deltaDist / deltaTheta;
            //Mise à jour de la position
            x += R * (-sin(theta) + sin(theta + deltaTheta));
            y += R * (cos(theta) - cos(theta + deltaTheta));
            // Mise à jour du cap
            theta += deltaTheta;

            // On limite le cap à +/- PI afin de ne pouvoir tourner dans les deux sens et pas dans un seul
            if (theta > PI) {
                theta -= 2 * PI;
            } else if (theta <= -PI) {
                theta += 2 * PI;
            }
        }
    } else {
        // TODO Vérifier qu'on ne perd pas l'accumulation dans ce mode
        printf("CG=%ld \t\tCD=%ld\r\n", compteurBrutG, compteurBrutD);
#ifdef LCD_ACTIVATE
        lcd.locate(0, 10);
        lcd.printf("L=%ld R=%ld", compteurBrutG, compteurBrutD);
#endif
    }
}

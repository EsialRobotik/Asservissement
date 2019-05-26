#include "Regulateur.h"

// Constructeur
Regulateur::Regulateur(bool isDistance) :
        filtreQuadRampDerivee(isDistance), filtrePid(isDistance)
{
    filtreQuadRampDeriveeON = true;
    accumulateur = 0;
    this->isDistance = isDistance;
}

// Destructeur
Regulateur::~Regulateur()
{
}

// Calcul de l'erreur
int64_t Regulateur::manage(int64_t consigne, int64_t feedback_odometrie)
{
    // On ajoute dans l'accumulateur la variation de déplacement mesuré par l'odométrie
    accumulateur += feedback_odometrie;

    // On filtre la consigne à l'aide de la QuadRampeDerivee pour savoir
    // si l'on est en phase d'accélération, constante ou de décélération.
    // La consigne en sortie est une consigne *de vitesse* !!!
    // Cette étape permet d'avoir un comportement linéaire au lieu de binaire
    // (= soit à l'arrêt, soit à fond) qui provoque des secouses et peut
    // renverser le robot. A la place, on accélère et déccélère tranquillement et ça, c'est beau :p
    int64_t consigneFiltree = filtreQuadRampDerivee.filtre(consigne, accumulateur, feedback_odometrie);


    // On calcul l'erreur, c'est à dire la différence entre la consigne à suivre
    // et la vitesse actuelle du robot
    int64_t erreur;
    bool enabled = filtreQuadRampDeriveeON;

    if (isDistance)
        enabled &= !Config::disableDistanceQuad;
    else
        enabled &= !Config::disableAngleQuad;

    if (enabled) {
        // feedback_odometrie *est* la vitesse actuelle
        erreur = consigneFiltree - feedback_odometrie;
    } else {
        // Pas de quadramp: on prend directement l'erreur de position
        // en entrée du PID. Attention aux secousses !
        erreur = consigne - accumulateur;
    }

    // On envoie l'erreur de vitesse (avec quadramp) ou de position (sans quadramp)
    // au PID qui va la filtrer par rapport aux paramètres du robot et aux coefficients d'asservissement
    // On obtient alors la vitesse à envoyer aux moteurs pour corriger l'erreur courante
    int64_t erreurFiltree = filtrePid.filtre(erreur);

    return erreurFiltree;
}

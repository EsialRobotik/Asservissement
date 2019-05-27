#include "QuadRampDerivee.h"

//--------------------------------------------------------
//                      QUADRAMPm
//-------------------------------------------------------
QuadRampDerivee::QuadRampDerivee(bool isDistance)
{
    if (isDistance) {
        // Dérivées premières de la consigne ( dans notre cas, la vitesse )
        derivee_1ier_pos = Config::DIST_QUAD_1ST_POS;
        derivee_1ier_neg = Config::DIST_QUAD_1ST_NEG;

        // Dérivées secondes de la consigne ( dans notre cas, l'accélération )
        derivee_2nd_pos_av = Config::DIST_QUAD_AV_2ND_ACC;
        derivee_2nd_neg_av = Config::DIST_QUAD_AV_2ND_DEC;

        //la même chose pour le déplacement vers l'arrière
        derivee_2nd_pos_ar = Config::DIST_QUAD_AR_2ND_ACC;
        derivee_2nd_neg_ar = Config::DIST_QUAD_AR_2ND_DEC;

        //On ajoute de l' "anticipation " pour éviter que la vitesse ne chute brutalement à zéro quand le point d'arrivée arrive....( bien pourri ca dis donc -_-" )
        gainAnticipation_av = Config::DIST_QUAD_AV_ANTICIPATION_GAIN_COEF;
        gainAnticipation_ar = Config::DIST_QUAD_AR_ANTICIPATION_GAIN_COEF;



        //Pour éviter les vibrations à l'arrivée
        tailleFenetreArrivee = Config::DIST_TAILLE_FENETRE_ARRIVEE;
    } else {
        // Dérivées premières de la consigne ( dans notre cas, la vitesse ) et le sens n'a pas d'importance en rotation
        derivee_1ier_pos = Config::ANGLE_QUAD_1ST_POS;
        derivee_1ier_neg = Config::ANGLE_QUAD_1ST_POS; // pas de NEG pour l'angle

        // Dérivées secondes de la consigne ( dans notre cas, l'accélération )
        derivee_2nd_pos_av = Config::ANGLE_QUAD_2ND_ACC;
        derivee_2nd_neg_av = Config::ANGLE_QUAD_2ND_DEC;

        //la même chose pour le déplacement vers l'arrière et le sens n'a pas d'importance en rotation
        derivee_2nd_pos_ar = Config::ANGLE_QUAD_2ND_ACC;
        derivee_2nd_neg_ar = Config::ANGLE_QUAD_2ND_DEC;

        //On ajoute de l' "anticipation " pour éviter que la vitesse ne chute brutalement à zéro quand le point d'arrivée arrive....( bien pourri ca dis donc -_-" )
        gainAnticipation_av = Config::ANGLE_QUAD_ANTICIPATION_GAIN_COEF;
        gainAnticipation_ar = Config::ANGLE_QUAD_ANTICIPATION_GAIN_COEF;

        //Pour éviter les vibrations à l'arrivée
        tailleFenetreArrivee = Config::ANGLE_TAILLE_FENETRE_ARRIVEE;
    }

    // La vitesse initiale est nulle
    prevConsigneVitesse = 0;
    // Comme il n'y a pas encore de consigne, on est arrivé
    arrivee = true;
}

// Destructeur
QuadRampDerivee::~QuadRampDerivee() {};

// On filtre la consigne pour prendre en compte l'accélération et la décélération
int64_t QuadRampDerivee::filtre(int64_t consigne, int64_t position_actuelle , int64_t vitesse)
{

    // Reset du flag "arrivee" signalant que la consigne est atteinte
    arrivee = false;

    //Calcul de la position du pivot qui sert à déterminer si l'on doit commencer à décélérer ou non
    char sens = (consigne - position_actuelle >= 0) ? 1 : -1;
    int64_t position_pivot;
    // Pas les mêmes coeffs suivant si on avance ou on recule, si le
    // robot est pas équilibré ; on prend la valeur de décélération, puisqu'on
    // calcule le début de la rampe de décélération
    if (sens == 1) {
        position_pivot = consigne + ((vitesse >= 0) ? -1 : 1) * (((vitesse * vitesse) / (2 * derivee_2nd_neg_av)) + llabs(vitesse) * gainAnticipation_av);
    } else {
        position_pivot = consigne + ((vitesse >= 0) ? -1 : 1) * (((vitesse * vitesse) / (2 * derivee_2nd_neg_ar)) + llabs(vitesse) * gainAnticipation_ar);
    }

    // Calcul de la consigne d'accélération qui dépend dans le sens dans lequel on roule
    // Pour l'accélération, on prend directement la valeur qui vient de config.h.
    // Pour la décélération, on calcule la force nécéssaire pour être
    // à l'arrêt au point de consigne, connaissant la vitesse de consigne
    // actuelle (v) et la distance restance (d).
    // La valeur nécessaire d'accélération est a = v²/(2*d)
    // Si le début de la rampe (pivot) est bien calculé, on doit avoir
    // à peu près la même valeur que la décélération du config.h
    int64_t acceleration_consign;
    if (sens == 1) {
        // on est en train d'avancer : si le pivot est devant nous, on
        // accélère, sinon on freine
        if (position_pivot >= position_actuelle) {
            // Accélération constante
            acceleration_consign = derivee_2nd_pos_av;
        } else {
            // Décélération permettant d'attendre la vitesse 0 au point de consigne
            acceleration_consign =
                - (prevConsigneVitesse * prevConsigneVitesse)
                    / (2 * llabs(consigne - position_actuelle));
        }
    } else {
        // on est en train de reculer : si le pivot est derrière nous, on
        // accélère, sinon on freine
        if (position_pivot <= position_actuelle) {
            // Accélération constante (vers l'arrière donc "-")
            acceleration_consign = -derivee_2nd_pos_ar;
        } else {
            // Décélération permettant d'attendre la vitesse 0 au point de consigne
            acceleration_consign =
                  (prevConsigneVitesse * prevConsigneVitesse)
                    / (2 * llabs(consigne - position_actuelle));
        }
    }

    // Calcul de la consigne de vitesse
    int64_t consigneVitesse = prevConsigneVitesse + acceleration_consign;
    // On borne la consigne, parce que faut éviter de trop déconner en atelier
    consigneVitesse = Utils::constrain(consigneVitesse, -derivee_1ier_neg, derivee_1ier_pos);
    // On stocke la nouvelle consigne pour l'itération suivante
    prevConsigneVitesse = consigneVitesse;

    //printf("consigne=%lld position_pivot=%lld\r\n", consigne, position_pivot);

    // On vérifie si on est dans la fenêtre d'arrivée et si oui, on est arrivé à la fin de la rampe
    if (llabs(consigne - position_actuelle) < tailleFenetreArrivee) {
        prevConsigneVitesse = 0; // On reset la consigne precedente
        arrivee = true;
        // On renvoie l'erreur non-filtrée
        return consigne - position_actuelle;
    }

    //On retourne la consigne de vitesse
    return consigneVitesse;
}

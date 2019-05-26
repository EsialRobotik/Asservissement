#ifndef PID
#define PID

#include "../Filtre.h"
#include "../../Utils/Utils.h"

// PID(Proportionnel-Integrale-Derivee) - Filtre sur l'erreur

// TODO configurable?
#define PID_INTEGRALE_BUFFER 8

class Pid : public Filtre
{
public:
    // Constructeur avec un booléen pour le type de PID (distance ou angle)
    Pid(bool isDistance);
    // Destructeur
    ~Pid();

    // On filtre l'erreur pour déterminer comment la corriger
    int64_t filtre(int64_t erreur);

private:
    // Buffer pour la somme des erreurs
    int64_t buf_integrale[PID_INTEGRALE_BUFFER];
    unsigned int integrale_index;

    // Ancienne erreur. Permet de calculer la dérivée de l'erreur
    int64_t old_erreur;

    //Coeffs d'asservissement
    int32_t kp, ki, kd; // Coeff venant de config.h

    //Ratios & Saturation
    /* Coeffs venant de config.h */
    double outRatio; // Ratio de sortie
    int32_t maxIntegral; // Valeur maximale de l'intégrale
    int32_t maxOutput; // Valeur maximale de la sortie

};

#endif

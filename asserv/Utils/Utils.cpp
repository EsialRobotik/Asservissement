#include "Utils.h"
#include "../odometrie/Odometrie.h"

//Limitation d'une valeur à un intervalle [valeurMin , valeurMax]
int64_t Utils::constrain(int64_t value , int64_t valeurMin , int64_t valeurMax)
{
    if (value < valeurMin)
        return valeurMin;

    if (value > valeurMax)
        return valeurMax;

    return value;
}

// Conversion mm vers UO
int64_t Utils::mmToUO(Odometrie *odo, double valeur)
{
	return (int64_t)((valeur  * (odo->getFrontParMetre()) * Config::uOParFront * 1.0) / 1000.0);
}

// Conversion U0 vers mm
double Utils::UOTomm(Odometrie *odo, int64_t valeur)
{
    return (double)((valeur * 1.0 / (odo->getFrontParMetre() * Config::uOParFront)) * 1000.0);
}

// Conversion degres en UO
int64_t Utils::degToUO(Odometrie *odo, double valeur)
{
    return (PI * valeur * odo->getDistanceRouesUO()) / 180;
}

// Conversion radians en UO
int64_t Utils::radToUO(Odometrie *odo, double valeur)
{
    return valeur * odo->getDistanceRouesUO();
}

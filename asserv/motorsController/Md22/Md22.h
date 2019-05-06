#if CONFIG_MOTORCTRL_MD22

#ifndef MD22
#define MD22

#include "../MotorsController.h"

// Instance d'une MD22
class Md22 : public MotorsController
{
public:

    // Constructeur
    Md22(PinName I2CsdaPin , PinName I2CsclPin);
    // Destructeur
    virtual ~Md22();

    // Paramétrage des vitesses
    virtual void setVitesseG(int vitMoteurG);
    virtual void setVitesseD(int vitMoteurD);
    virtual int getVitesseG(void) { return vitMoteurG; }
    virtual int getVitesseD(void) { return vitMoteurD; }

private:
    int vitMoteurG, vitMoteurD;
    // Communication I2C entre la Mbed et la MD22
    I2C i2cLink;
};

#endif /* MD22 */

#endif /* CONFIG_MOTORCTRL_MD22 */

/*
 * Md25ctrl.h
 *
 *  Created on: 12 mars 2017
 *      Author: pmx
 */

#if CONFIG_MOTORCTRL_MD25

#ifndef ASSERV_MOTORSCONTROLLER_MD25_MD25CTRL_H_
#define ASSERV_MOTORSCONTROLLER_MD25_MD25CTRL_H_

#include <I2C.h>
#include <PinNames.h>

#include "../MotorsController.h"
#include "MD25.h"

#define ADDR_MD25 0xB0

class Md25ctrl: public MotorsController
{
public:

    // Constructeur
    Md25ctrl(PinName I2CsdaPin, PinName I2CsclPin);
    // Destructeur
    virtual ~Md25ctrl();

    // Paramétrage des vitesses
    virtual void setVitesseG(int);
    virtual void setVitesseD(int);
    virtual int getVitesseG()
    {
        return vitMoteurG_;
    }
    virtual int getVitesseD()
    {
        return vitMoteurD_;
    }

private:
    int vitMoteurG_, vitMoteurD_;
    // Communication I2C entre la Mbed et la MD22
    I2C i2cLink_;

    MD25 md_;

    bool connected_;

};

#endif /* ASSERV_MOTORSCONTROLLER_MD25_MD25CTRL_H_ */

#endif /* CONFIG_MOTORCTRL_MD25 */

#ifndef QIK
#define QIK

#include "../MotorsController.h"

// instance of a Qik2s9v1
class Qik : public MotorsController
{

public :
    Qik(PinName txPinToQik, PinName rxPinToQik);
    ~Qik();

    virtual void setVitesseG(int vitMoteurG);
    virtual void setVitesseD(int vitMoteurD);

private :

    Serial qikSerial;

};

#endif

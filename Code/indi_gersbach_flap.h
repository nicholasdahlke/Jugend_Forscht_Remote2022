#pragma once

#include "libindi/defaultdevice.h"
#include "indipropertyswitch.h"
#include "libindi/connectionplugins/connectionserial.h"
#include "indidustcapinterface.h"
#include "indicom.h"

#include <termios.h>



namespace Connection
{
    class Serial;
}

class IndiGersbachFlap : public INDI::DefaultDevice, public INDI::DustCapInterface
{
public:
    IndiGersbachFlap();
    virtual ~IndiGersbachFlap() = default;
    virtual bool initProperties() override;
    virtual const char *getDefaultName() override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    virtual bool updateProperties() override;
    virtual IPState ParkCap() override;
    virtual IPState  UnParkCap() override;

private:
    bool Handshake();
    bool sendCommand(const char *cmd);
    int PortFD { -1 };

    Connection::Serial *serialConnection{nullptr};
};

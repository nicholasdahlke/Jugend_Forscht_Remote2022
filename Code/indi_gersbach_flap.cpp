#include "config.h"
#include "indi_gersbach_flap.h"

//We declare an auto pointer to IndiGersbachFlap
static std::unique_ptr<IndiGersbachFlap> driver(new IndiGersbachFlap());

IndiGersbachFlap::IndiGersbachFlap()
{
    setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);
}

const char *IndiGersbachFlap::getDefaultName()
{
    return "Gersbach CDK Deckel";
}

bool IndiGersbachFlap::initProperties()
{
    INDI::DefaultDevice::initProperties();

    addAuxControls();

    serialConnection = new Connection::Serial(this);
    serialConnection->registerHandshake([&]() {return Handshake();});
    serialConnection->setDefaultBaudRate(Connection::Serial::B_115200);
    serialConnection->setDefaultPort("/dev/ttyACM1");
    registerConnection(serialConnection);

    initDustCapProperties(getDeviceName(), MAIN_CONTROL_TAB);
    setDriverInterface(AUX_INTERFACE | DUSTCAP_INTERFACE);
    return true;
}

bool IndiGersbachFlap::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
        defineProperty(&ParkCapSP);
    } else
    {
        deleteProperty(ParkCapSP.name);
    }
    return true;
}

bool IndiGersbachFlap::ISNewSwitch(const char *dev, const char *name, ISState *states, char **names, int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (processDustCapSwitch(dev, name, states, names, n))
            return true;
    }
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool IndiGersbachFlap::Handshake()
{
    if (isSimulation())
    {
        LOGF_INFO("Connected successfuly to simulated %s.", getDeviceName());
        return true;
    }
    PortFD = serialConnection->getPortFD();
    return true;
}

bool IndiGersbachFlap::sendCommand(const char *cmd)
{
    int nbytes_written = 0, tty_rc = 0;
    LOGF_DEBUG("CMD <%s>", cmd);

    if (!isSimulation())
    {
        tcflush(PortFD, TCIOFLUSH);
        if ((tty_rc = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
        {
            char errorMessage[MAXRBUF];
            tty_error_msg(tty_rc, errorMessage, MAXRBUF);
            LOGF_ERROR("Serial write error: %s", errorMessage);
            return false;
        }
    }
    return true;
}

IPState IndiGersbachFlap::UnParkCap()
{
    char command_open = 'd';
    LOG_INFO("Open Command send");
    sendCommand(&command_open);
    return IPS_OK;
}

IPState  IndiGersbachFlap::ParkCap()
{
    char command_close = 'e';
    LOG_INFO("Close Command send");
    sendCommand(&command_close);
    return IPS_OK;
}

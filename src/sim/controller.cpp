#include "controller.h"

#include "foc.h"
#include "params.h"
#include "pwmgeneration.h"

// Globals provided by the test stubs.
extern volatile uint16_t g_input_angle;
extern volatile double g_il1_input;
extern volatile double g_il2_input;
extern volatile bool disablePWM;

namespace sim
{
void Controller::SetRotorAngle(uint16_t angle)
{
    g_input_angle = angle;
}

void Controller::SetCurrentInputs(double il1, double il2)
{
    g_il1_input = il1;
    g_il2_input = il2;
}

void Controller::SetTorquePercent(float torque)
{
    PwmGeneration::SetTorquePercent(torque);
}

void Controller::SetOpmode(int opmode)
{
    PwmGeneration::SetOpmode(opmode);
}

void Controller::Run()
{
    PwmGeneration::Run();
}

bool Controller::PwmEnabled() const
{
    return !disablePWM;
}

double Controller::UdVolts(double vdc) const
{
    return (vdc / 65536.0) * Param::GetFloat(Param::ud);
}

double Controller::UqVolts(double vdc) const
{
    return (vdc / 65536.0) * Param::GetFloat(Param::uq);
}

double Controller::Id() const
{
    return Param::GetFloat(Param::id);
}

double Controller::Iq() const
{
    return Param::GetFloat(Param::iq);
}

double Controller::Ifw() const
{
    return Param::GetFloat(Param::ifw);
}
} // namespace sim

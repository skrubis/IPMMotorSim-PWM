#ifndef SIM_CONTROLLER_H
#define SIM_CONTROLLER_H

#include <cstdint>

namespace sim
{
class Controller
{
public:
    void SetRotorAngle(uint16_t angle);
    void SetCurrentInputs(double il1, double il2);
    void SetTorquePercent(float torque);
    void SetOpmode(int opmode);
    void Run();
    bool PwmEnabled() const;

    double UdVolts(double vdc) const;
    double UqVolts(double vdc) const;
    double Id() const;
    double Iq() const;
    double Ifw() const;
};
} // namespace sim

#endif // SIM_CONTROLLER_H

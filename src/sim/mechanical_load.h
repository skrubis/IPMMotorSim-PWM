#ifndef SIM_MECHANICAL_LOAD_H
#define SIM_MECHANICAL_LOAD_H

namespace sim
{
class MechanicalLoad
{
public:
    void Step(double torque, double dt)
    {
        (void)torque;
        (void)dt;
    }
};
} // namespace sim

#endif // SIM_MECHANICAL_LOAD_H

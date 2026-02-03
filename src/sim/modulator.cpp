#include "modulator.h"

#include "foc.h"

namespace sim
{
DutyCycles Modulator::GetDutyCycles() const
{
    DutyCycles duty;
    duty.a = FOC::DutyCycles[0];
    duty.b = FOC::DutyCycles[1];
    duty.c = FOC::DutyCycles[2];
    duty.a_norm = duty.a / 65535.0;
    duty.b_norm = duty.b / 65535.0;
    duty.c_norm = duty.c / 65535.0;
    return duty;
}
} // namespace sim

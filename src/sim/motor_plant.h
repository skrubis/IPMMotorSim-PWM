#ifndef SIM_MOTOR_PLANT_H
#define SIM_MOTOR_PLANT_H

#include "motormodel.h"

namespace sim
{
class MotorPlant : public MotorModel
{
public:
    using MotorModel::MotorModel;
};
} // namespace sim

#endif // SIM_MOTOR_PLANT_H

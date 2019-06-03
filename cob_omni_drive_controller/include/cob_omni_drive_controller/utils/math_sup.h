#pragma once

namespace MathSup
{
void normalizePi(double& val);

double atan4quad(double y, double x);

double getWeightedDelta(double current_position, double old_target, double new_target);
double limitValue(double value, double limit);

}  // namespace MathSup

#include "braitenberg_vehicle.h"

BraitenbergVehicle::BraitenbergVehicle()
: type_(TYPE_A)
, factor1_(1.0)
, factor2_(0.0)
{
}

BraitenbergVehicle::BraitenbergVehicle(Type type, float factor1, float factor2)
: type_(type)
, factor1_(factor1)
, factor2_(factor2)
{
}

/** Depending on the Braitenberg type, the input is mapped to the output.
  * The factor1 and factor2 are used to scale the output values. And the input values
  * are normalized by dividing them through 4, because that is the maximum input value. */
void BraitenbergVehicle::computeWheelSpeeds(float left_in, float right_in, float& left_out, float& right_out)
{
  switch (this->type_)
  {
    case TYPE_A:
      left_out = this->factor1_ * left_in;
      right_out = this->factor1_ * right_in;
      break;
    case TYPE_B:
      left_out = this->factor1_ * right_in;
      right_out = this->factor1_ * left_in;
      break;
    case TYPE_C:
      left_out = this->factor1_ * left_in + this->factor2_ * right_in;
      right_out = this->factor1_ * right_in + this->factor2_ * left_in;
      break;
  }
}

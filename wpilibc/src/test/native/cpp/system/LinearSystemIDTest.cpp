#include <gtest/gtest.h>

#include <frc/system/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>

#include <units/units.h>

TEST(LinearSystemIDTest, IdentifyVelocitySystem) {
  double kv = 1.0;
  double ka = 0.5;
  auto system = frc::IdentifyVelocitySystem(kv, ka, 12_V);
}
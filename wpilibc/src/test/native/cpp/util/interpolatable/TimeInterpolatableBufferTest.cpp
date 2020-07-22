#include "frc/util/interpolatable/TimeInterpolatableBuffer.h"
#include "gtest/gtest.h"
#include <units/time.h>
#include "frc/geometry/Rotation2d.h"

TEST(TimeInterpolatableBufferTest, TestInterpolation) {
    frc::TimeInterpolatableBuffer<frc::Rotation2d> buffer{};

    buffer.addSample(0_s, frc::Rotation2d(0_rad));
    EXPECT_TRUE(buffer.getSample(0_s) == frc::Rotation2d(0_rad));
    buffer.addSample(1_s, frc::Rotation2d(1_rad));
    EXPECT_TRUE(buffer.getSample(0.5_s) == frc::Rotation2d(0.5_rad));
    EXPECT_TRUE(buffer.getSample(1_s) == frc::Rotation2d(1_rad));
    buffer.addSample(3_s, frc::Rotation2d(2_rad));
    EXPECT_TRUE(buffer.getSample(2_s) == frc::Rotation2d(1.5_rad));
}
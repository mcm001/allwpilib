/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.math.interpolation.interpolation;

import edu.wpi.first.wpiutil.math.MathUtil;

public class InterpolatingDouble implements Interpolatable<Double> {

    public double value;

    public InterpolatingDouble(double value) {
        this.value = value;
    }

    @Override
    public Double interpolate(Double endValue, double t) {
        return value + (endValue - value) * MathUtil.clamp(t, 0, 1);
    }
}

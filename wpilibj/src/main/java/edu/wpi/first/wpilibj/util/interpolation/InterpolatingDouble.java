package edu.wpi.first.wpilibj.util.interpolation;

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

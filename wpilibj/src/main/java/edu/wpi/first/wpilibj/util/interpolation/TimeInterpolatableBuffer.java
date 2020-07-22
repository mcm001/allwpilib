/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.util.interpolation;

import java.util.TreeMap;

/**
 * The TimeInterpolatableBuffer provides an easy way to estimate past measurements. One
 * application might be in conjunction with the
 * {@link edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator},
 * where knowledge of the robot pose at the time when vision or other global measurement
 * were recorded is necessary, or for recording the past angles of mechanisms as measured
 * by encoders. Currently, the {@link edu.wpi.first.wpilibj.geometry.Pose2d},
 * {@link edu.wpi.first.wpilibj.geometry.Rotation2d}, {@link edu.wpi.first.wpilibj.geometry.Translation2d}
 * and {@link InterpolatingDouble} classes implement {@link Interpolatable}.
 * @param <T>
 */
public class TimeInterpolatableBuffer<T extends Interpolatable<T>> {

    private final double historySize;
    private TreeMap<Double, T> bufferMap = new TreeMap<>();

    public TimeInterpolatableBuffer(double historySizeSeconds) {
        this.historySize = historySizeSeconds;
    }

    public void addSample(double timeSeconds, T sample) {
        cleanUp(timeSeconds);
        bufferMap.put(timeSeconds, sample);
    }

    private void cleanUp(double time) {
        while (!bufferMap.isEmpty()) {
            var entry = bufferMap.lastEntry();
            if (time - entry.getKey() >= historySize) {
                bufferMap.remove(entry.getKey());
            } else {
                return;
            }
        }
    }

    public void clear() {
        bufferMap.clear();
    }

    /**
     * Sample the buffer at the given time. If the buffer is empty, this will
     * return null.
     *
     * @param timeSeconds The time at which to sample.
     * @return The interpolated value at that timestamp. Might be null.
     */
    public T getSample(double timeSeconds) {
        if(bufferMap.isEmpty()) return null;

        // Special case for when the requested time is the same as a sample
        var nowEntry = bufferMap.get(timeSeconds);
        if(nowEntry != null) return nowEntry;

        var topBound = bufferMap.ceilingEntry(timeSeconds);
        var bottomBound  = bufferMap.floorEntry(timeSeconds);

        // Return null if neither sample exists, and the opposite bound if the other is null
        if(topBound == null && bottomBound == null) return null;
        if(topBound == null) return bottomBound.getValue();
        if(bottomBound == null) return topBound.getValue();

        // Otherwise, interpolate. Because T is between [0, 1], we want the ratio of (the difference between
        // the current time and bottom bound) and (the difference between top and bottom bounds).
        else {
            return bottomBound.getValue().interpolate(topBound.getValue(),
                (timeSeconds - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey()));
        }
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.wpilibStateSpace;

import edu.wpi.first.wpilibj.math.DrakeJNI;

public final class DevMain {
    private DevMain() {
    }

    /**
     * Main entry point.
     */
    public static void main(String[] args) {
        System.out.println("Hello World!");
        try {
            DrakeJNI.forceLoad();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}

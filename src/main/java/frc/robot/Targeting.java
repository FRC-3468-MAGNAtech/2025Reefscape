// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LimeLightConstants;
import frc.robot.Subsystems.LimelightHelpers;

/** Add your docs here. */
public class Targeting {


    public static double alignToReedLeft() {
        double tx = LimelightHelpers.getTX("null");
        return LimeLightConstants.llPIDctrlDrive.calculate(tx);
    }

    public static double alignToReedRight() {
        double tx = LimelightHelpers.getTX("null");
        return LimeLightConstants.llPIDctrlDrive.calculate(tx);
    }
}

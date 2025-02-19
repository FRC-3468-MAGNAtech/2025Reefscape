// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LimeLightConstants;
import frc.robot.Subsystems.LimelightHelpers;

/** Add your docs here. */
public class Targeting {


    public static double alignToReedLeft() {
        double tx = LimelightHelpers.getTX("limelight-right");
        return LimeLightConstants.llPIDctrlStraifLeft.calculate(tx);
    }

    public static double alignToReedRight() {
        double tx = LimelightHelpers.getTX("limelight-left");
        return LimeLightConstants.llPIDctrlStraifRight.calculate(tx);
    }

    public static double driveToReedLeft() {
        double ta = LimelightHelpers.getTA("limelight-right");
        return LimeLightConstants.llPIDctrlDriveLeft.calculate(ta);
    }

    public static double driveToReedRight() {
        double ta = LimelightHelpers.getTA("limelight-left");
        return LimeLightConstants.llPIDctrlDriveRight.calculate(ta);
    }
}

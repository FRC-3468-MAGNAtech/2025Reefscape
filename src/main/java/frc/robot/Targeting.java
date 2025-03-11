// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LimeLightConstants;
import frc.robot.Subsystems.LimelightHelpers;

/** Add your docs here. */
public class Targeting {

    // L1 through L3 commands
    public static double alignToReed(boolean left) {
        double tx = LimelightHelpers.getTX("limelight-front");
        if (left) {
            return LimeLightConstants.llPIDctrlStraifLeft.calculate(tx);
        } else {
            return LimeLightConstants.llPIDctrlStraifRight.calculate(tx);
        }
    }

    public static double driveToReed(boolean left) {
        double ta = LimelightHelpers.getTA("limelight-front");
        if (left) {
            return LimeLightConstants.llPIDctrlDriveLeft.calculate(ta);
        } else {
            return LimeLightConstants.llPIDctrlDriveRight.calculate(ta);
        }
    }

    // L4 specific commands
    public static double l4alignToReed(boolean left) {
        double tx = LimelightHelpers.getTX("limelight-back");
        if (left) {
            return LimeLightConstants.llPIDctrlStraifLeft.calculate(tx);
        } else {
            return LimeLightConstants.llPIDctrlStraifRight.calculate(tx);
        }
    }

    public static double l4driveToReed(boolean left) {
        double ta = LimelightHelpers.getTA("limelight-back");
        if (left) {
            return LimeLightConstants.llPIDctrlDriveLeft.calculate(ta);
        } else {
            return LimeLightConstants.llPIDctrlDriveRight.calculate(ta);
        }
    }
    
    // failed algea pickup commands
    public static double driveToAlgae() {
        double ta = LimelightHelpers.getTA("limelight-forward");
        return LimeLightConstants.llPIDctrlAlgaeDrive.calculate(ta);
    }
    public static double rotToAlgae() {
        double ty = LimelightHelpers.getTY("limelight-forward");
        return LimeLightConstants.llPIDctrlAlgaeRot.calculate(ty);
    }
    public static double alignToAlgae() {
        double tx = LimelightHelpers.getTX("limelight-forward");
        return LimeLightConstants.llPIDctrlAlgaeAlign.calculate(tx);
    }


}

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
        
        if (left) {
            double tx = LimelightHelpers.getTX("limelight-left");
            return LimeLightConstants.llPIDctrlStraifLeft.calculate(tx);
        } else {
            double tx = LimelightHelpers.getTX("limelight-right");
            return LimeLightConstants.llPIDctrlStraifRight.calculate(tx);
        }
    }

    public static double driveToReed(boolean left) {
        if (left) {
            double ta = LimelightHelpers.getTA("limelight-left");
            return LimeLightConstants.llPIDctrlDriveLeft.calculate(ta);
        } else {
            double ta = LimelightHelpers.getTA("limelight-right");
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

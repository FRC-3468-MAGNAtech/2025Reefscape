// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import frc.robot.Subsystems.LimelightHelpers;

/** Add your docs here. */
public class Camera {
    /**
     * Updates the pose of the limelight indicated by limelightName
     * @param limelightName The name of the limelight
     * @param m_PoseEstimator The pose estimator (typically from the swerve subsystem)
     * @param yawRate The angular velocity of the robot
     */
    // public static void UpdateLimelight(String limelightName, PoseEstimator m_PoseEstimator, double yawRate) {
    //     LimelightHelpers.SetRobotOrientation(limelightName, m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0,0,0,0,0);
    //     LimelightHelpers.PoseEstimate m_estimate;
    //     if (RobotContainer.isRedAlliance())
    //         m_estimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
    //     else
    //         m_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    //     if (Math.abs(yawRate) > 270 || m_estimate.tagCount == 0)
    //         return;
    //     m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    //     m_PoseEstimator.addVisionMeasurement(
    //         m_estimate.pose,
    //         m_estimate.timestampSeconds);
    // }
}

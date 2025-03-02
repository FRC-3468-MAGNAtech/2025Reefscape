// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {
  private final SparkMax armMotor;
  private final SparkClosedLoopController armController;
  private final ArmFeedforward feedForward;
  private final AbsoluteEncoder armEncoder;
  private final SoftLimitConfig forwardLimit;
  private final SoftLimitConfig backwardLimit;
  
  public Arm() {
    armMotor = new SparkMax(ArmConstants.armID, MotorType.kBrushless);
    forwardLimit = new SoftLimitConfig().forwardSoftLimit(45);
    forwardLimit.forwardSoftLimitEnabled(true);
    backwardLimit = new SoftLimitConfig().reverseSoftLimit(-45);
    backwardLimit.reverseSoftLimitEnabled(true);

    SparkMaxConfig config = new SparkMaxConfig();
    armEncoder = armMotor.getAbsoluteEncoder();
    config.absoluteEncoder.zeroCentered(true);

    config.idleMode(IdleMode.kBrake);
    config.closedLoop.pid(0, 0, 0);
    config.apply(backwardLimit);
    config.apply(forwardLimit);
    config.inverted(true);
    config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
    
    armController = armMotor.getClosedLoopController();
    armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    feedForward = new ArmFeedforward(0.5, 1, 1.58);
  }

  public void front() {
    armMotor.set(ArmConstants.armForward);
  }

  public void back() {
    armMotor.set(ArmConstants.armBackward);
  }
  
  public void stop() {
    armMotor.set(0);
  }

  public void PointMove(double position) {
    position = armEncoder.getPosition() * 360;
    if (position < 0.1){
      position = 0.1;
    }
    armController.setReference(position - ArmConstants.armOffSet, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward.calculate(position, 0));
   
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm angle", armEncoder.getPosition());
    SmartDashboard.putNumber("output", armMotor.getAppliedOutput());
    // This method will be called once per scheduler run
  }
}

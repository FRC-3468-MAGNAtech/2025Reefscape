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
import frc.robot.Constants.ElevConstants;


public class Arm extends SubsystemBase {
  private SparkMax armMotor;
  private SparkClosedLoopController armController;
  private ArmFeedforward feedForward;
  private AbsoluteEncoder armEncoder;
  private SoftLimitConfig forwardLimit;
  private SoftLimitConfig backwardLimit;
  private double position;
  
  public Arm() {
    armMotor = new SparkMax(ArmConstants.armID, MotorType.kBrushless);
    forwardLimit = new SoftLimitConfig().forwardSoftLimit(83);
    forwardLimit.forwardSoftLimitEnabled(true);
    backwardLimit = new SoftLimitConfig().reverseSoftLimit(-120);
    backwardLimit.reverseSoftLimitEnabled(true);

    SparkMaxConfig config = new SparkMaxConfig();
    armEncoder = armMotor.getAbsoluteEncoder();
    config.absoluteEncoder.zeroCentered(true);

    config.idleMode(IdleMode.kBrake);
    config.closedLoop.pid( ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
    config.apply(backwardLimit);
    config.apply(forwardLimit);
    config.inverted(true);
    
    armController = armMotor.getClosedLoopController();
    armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    feedForward = new ArmFeedforward(ArmConstants.ffKS, ArmConstants.ffKG, ArmConstants.ffKV);
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

  public void pointMove(double angle) {
    position = angle;
    armController.setReference(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward.calculate(position, 0));
  }

  public void armStay() {
    position = armEncoder.getPosition();
    armController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot1, feedForward.calculate(position, 0));
  }

  public boolean isAtSetpoint() {
    return ((Math.abs(position-armEncoder.getPosition())) <= ArmConstants.tolerance);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm angle", armEncoder.getPosition());
    SmartDashboard.putNumber("output", armMotor.getAppliedOutput());
    // This method will be called once per scheduler run
  }
}

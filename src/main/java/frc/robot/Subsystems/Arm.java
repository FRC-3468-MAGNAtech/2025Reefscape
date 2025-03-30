// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {
  private SparkMax armMotor;
  private SparkClosedLoopController armController;
  private ArmFeedforward feedForward;
  //private ArmFeedForward feedForward2;
  private AbsoluteEncoder armEncoder;
  private SoftLimitConfig forwardLimit;
  private SoftLimitConfig backwardLimit;
  private double position;
  
  public Arm() {
    armMotor = new SparkMax(ArmConstants.armID, MotorType.kBrushless);
    forwardLimit = new SoftLimitConfig().forwardSoftLimit(ArmConstants.forwardSoftLimit);
    forwardLimit.forwardSoftLimitEnabled(true);
    backwardLimit = new SoftLimitConfig().reverseSoftLimit(ArmConstants.reverseSoftLimit);
    backwardLimit.reverseSoftLimitEnabled(true);

    SparkMaxConfig config = new SparkMaxConfig();
    armEncoder = armMotor.getAbsoluteEncoder();
    config.absoluteEncoder.zeroCentered(true);
    config.absoluteEncoder.positionConversionFactor(360.0);
    
    config.idleMode(IdleMode.kBrake);
    config.closedLoop.pid( ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
    config.closedLoopRampRate(0.5);
    config.apply(backwardLimit);
    config.apply(forwardLimit);
    config.inverted(true);
    
    armController = armMotor.getClosedLoopController();
    armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    

    feedForward = new ArmFeedforward(ArmConstants.ffKS, ArmConstants.ffKG, ArmConstants.ffKV);
    //feed
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
    position = -angle;
    armController.setReference(angle , ControlType.kPosition, ClosedLoopSlot.kSlot0, ArmConstants.ff * Math.sin(armEncoder.getPosition()),ArbFFUnits.kPercentOut);
  }

  public void armStay() {
    position = armEncoder.getPosition() ;//* 360;
        armController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot1, ArmConstants.ff * Math.sin(armEncoder.getPosition()),ArbFFUnits.kPercentOut);
  }

  public boolean isAtSetpoint() {
    return ((Math.abs(position-armEncoder.getPosition())) <= ArmConstants.tolerance);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm angle", armEncoder.getPosition());
    SmartDashboard.putNumber("Arm Set Pos", position);
    SmartDashboard.putNumber("output", armMotor.getAppliedOutput());
    SmartDashboard.putNumber("Applied Voltage", armMotor.getOutputCurrent());
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  
  private final SparkMax armMotor;
  private final SparkClosedLoopController armController;
  private final ArmFeedforward feedForward;
  private final SparkAbsoluteEncoder armEncoder;
  private final SoftLimitConfig forwardLimit;
  private final SoftLimitConfig backwardLimit;
  

  public Arm() {
    armMotor = new SparkMax(ArmConstants.armID, MotorType.kBrushless);
    forwardLimit = new SoftLimitConfig().forwardSoftLimit(0);
    forwardLimit.forwardSoftLimitEnabled(true);
    backwardLimit = new SoftLimitConfig().reverseSoftLimit(0);
    backwardLimit.reverseSoftLimitEnabled(true);

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    armEncoder = armMotor.getAbsoluteEncoder();
    //config.closedLoop.pid(0, 0, 0);
    
    armController = armMotor.getClosedLoopController();
    armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    feedForward= new ArmFeedforward(0, 0, 0);
  }

  public void front() {
    armMotor.set(ArmConstants.armForward);
  }

  public void back() {
    armMotor.set(ArmConstants.armBackward);
  }

    public void PointMove(double position) {
    if (position < 0.1){
      position = 0.1;
    }
    armController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    armController.
    // This method will be called once per scheduler run
  }
}

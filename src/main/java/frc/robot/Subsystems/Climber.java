
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevConstants;


public class Climber extends SubsystemBase {
  private SparkMax climbMtr;
  private SparkClosedLoopController climbLoopController;
  private SoftLimitConfig forwardLimit;
  private SoftLimitConfig backwardLimit;
  private SparkLimitSwitch climbSwitch;
  private SparkClosedLoopController eLoopController;
  private RelativeEncoder climbEncoder;
  private double position;


  public Climber() {
    climbMtr = new SparkMax(ClimberConstants.climbID, MotorType.kBrushless);

    forwardLimit = new SoftLimitConfig().forwardSoftLimit(ClimberConstants.forwardSoftLimit);
    forwardLimit.forwardSoftLimitEnabled(true);
    backwardLimit = new SoftLimitConfig().reverseSoftLimit(ClimberConstants.reverseSoftLimit);
    backwardLimit.reverseSoftLimitEnabled(true);

    SparkMaxConfig conf = new SparkMaxConfig();
    climbEncoder = climbMtr.getEncoder();

    conf.idleMode(IdleMode.kBrake);
    conf.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    conf.closedLoop.pid(ClimberConstants.climbP, ClimberConstants.climbI, ClimberConstants.climbD);
    conf.apply(backwardLimit);
    conf.apply(forwardLimit);
    conf.limitSwitch.forwardLimitSwitchEnabled(false);
    conf.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed);
    
    climbMtr.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    eLoopController = climbMtr.getClosedLoopController();
    climbEncoder = climbMtr.getEncoder();
    climbSwitch = climbMtr.getForwardLimitSwitch();

    climbLoopController = climbMtr.getClosedLoopController(); 
  }

  public boolean limitSwitch() {
    return climbSwitch.isPressed();
  }

  public void climbZero() {
    climbEncoder.setPosition(0.1);
  }

  public void go() {
    climbMtr.set(ClimberConstants.riseSpeed); 
  }

  public void goDown() {
    climbMtr.set(ClimberConstants.lowerSpeed);
  }

  public void stop() {
    climbMtr.set(0);
  }

  public void pointMove(double location) {
    position = location;
    climbLoopController.setReference(location, ControlType.kPosition);
  }

  public boolean isAtSetpoint() {
    return ((Math.abs(position-climbEncoder.getPosition())) <= ElevConstants.tolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevConstants;


public class Evelator extends SubsystemBase {
  private SparkMax elevMtr1;
  private SparkMax elevMtr2;
  private SparkClosedLoopController eLoopController;
  private RelativeEncoder elevEncoder;
  private SoftLimitConfig elevBottomLimit;
  private SoftLimitConfig elevTopLimit;
  private SparkLimitSwitch elevSwitch;
  private ElevatorFeedforward feedForward;
  private ElevatorFeedforward feedForward2;
  private double position;


  public Evelator() {
    elevMtr1 = new SparkMax(ElevConstants.elev1ID, MotorType.kBrushless);
    elevMtr2 = new SparkMax(ElevConstants.elev2ID, MotorType.kBrushless);
    SparkMaxConfig conf = new SparkMaxConfig();
    SparkMaxConfig conf2 = new SparkMaxConfig();
    
    elevTopLimit = new SoftLimitConfig().forwardSoftLimit(ElevConstants.tLimit);
    elevTopLimit.forwardSoftLimitEnabled(true);
    elevBottomLimit = new SoftLimitConfig().reverseSoftLimit(ElevConstants.bLimit);
    elevBottomLimit.reverseSoftLimitEnabled(true);

    conf.idleMode(IdleMode.kBrake);
    conf.closedLoop.pid(ElevConstants.elevP, ElevConstants.elevI, ElevConstants.elevD);
    conf.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    conf.smartCurrentLimit(40);

    conf.closedLoopRampRate(1.1);
    conf.limitSwitch.forwardLimitSwitchEnabled(false);
    conf.limitSwitch.forwardLimitSwitchType(Type.kNormallyOpen);
    conf.apply(elevTopLimit);
    conf.apply(elevBottomLimit);

    conf2.follow(ElevConstants.elev1ID, false);
    
    elevMtr1.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    elevMtr2.configure(conf2, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    // elevBottomLimit = elevMtr1.getReverseLimitSwitch();
    eLoopController = elevMtr1.getClosedLoopController();
    elevEncoder = elevMtr1.getEncoder();
    elevSwitch = elevMtr1.getForwardLimitSwitch();

    feedForward = new ElevatorFeedforward(ElevConstants.ffKS, ElevConstants.ffKG, ElevConstants.ffKV);
    feedForward2 = new ElevatorFeedforward(ElevConstants.ffKS2, ElevConstants.ffKG2, ElevConstants.ffKV2);
  }

  public void elevUp() {
    elevMtr1.set(ElevConstants.elevUp);
  }

  public void elevDown() {
    elevMtr1.set(ElevConstants.elevDown);
  }

  public void elevStop() {
    elevMtr1.set(0);
  }

  public boolean limitSwitch() {
    return elevSwitch.isPressed();
  }

  public void elevZero() {
    elevEncoder.setPosition(0.1);
  }

  public void elevStay() {
    position = elevEncoder.getPosition();
    eLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot1, feedForward2.calculate(position));
  }
  
  public void pointMove(double height) {
    position = height;
    eLoopController.setReference(height, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward2.calculate(position));
  }

  public boolean isAtSetpoint() {
    return ((Math.abs(position-elevEncoder.getPosition())) <= ElevConstants.tolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator position", elevEncoder.getPosition());
    SmartDashboard.putNumber("Elev Set Pos", position);
    SmartDashboard.putBoolean("elev limit", limitSwitch());
  }
}

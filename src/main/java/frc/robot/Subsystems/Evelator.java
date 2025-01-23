// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevConstants;

public class Evelator extends SubsystemBase {
  
private final SparkMax elevMtr1;
private final SparkMax elevMtr2;
private final SparkClosedLoopController eLoopController;
private final SparkLimitSwitch elevBottomLimit;

  public Evelator() {
    elevMtr1 = new SparkMax(ElevConstants.elev1ID, MotorType.kBrushless);
    elevMtr2 = new SparkMax(ElevConstants.elev2ID, MotorType.kBrushless);

    SparkMaxConfig conf = new SparkMaxConfig();
    conf.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    conf.closedLoop.pid(ElevConstants.elevP, ElevConstants.elevI, ElevConstants.elevD);
    conf.limitSwitch.reverseLimitSwitchEnabled(true);
    conf.idleMode(IdleMode.kBrake);
    elevMtr1.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig conf2 = new SparkMaxConfig();
    conf2.follow(ElevConstants.elev2ID, true);
    elevMtr2.configure(conf2, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    
    elevBottomLimit = elevMtr1.getReverseLimitSwitch();
    eLoopController = elevMtr1.getClosedLoopController();
  }

  public void setAngle(double rotations) {
    eLoopController.setReference(rotations, ControlType.kPosition);
  }

  public void ElevUp() {
    elevMtr1.set(ElevConstants.elevUp);
  }

  public void ElevDown() {
    elevMtr1.set(ElevConstants.elevDown);
  }

  public void ElevStop() {
    elevMtr1.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

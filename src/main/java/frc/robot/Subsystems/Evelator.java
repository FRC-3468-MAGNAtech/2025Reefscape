// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.revrobotics.AbsoluteEncoder;
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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevConstants;

public class Evelator extends SubsystemBase {
  
private final SparkMax elevMtr1;
private final SparkMax elevMtr2;
private final SparkClosedLoopController eLoopController;
private final ProfiledPIDController ePIDController;
private final AbsoluteEncoder elevEncoder;
private final SparkLimitSwitch elevBottomLimit;
private final TrapezoidProfile elevProfile;
private TrapezoidProfile.State goal;
private TrapezoidProfile.State current;

  public Evelator() {
    elevMtr1 = new SparkMax(ElevConstants.elev1ID, MotorType.kBrushless);
    elevMtr2 = new SparkMax(ElevConstants.elev2ID, MotorType.kBrushless);
    elevProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
    goal = new TrapezoidProfile.State();
    current = new TrapezoidProfile.State();

    SparkMaxConfig conf = new SparkMaxConfig();
    conf.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    conf.closedLoop.pid(ElevConstants.elevP, ElevConstants.elevI, ElevConstants.elevD);
    conf.limitSwitch.reverseLimitSwitchEnabled(true);
    conf.idleMode(IdleMode.kBrake);
    elevMtr1.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig conf2 = new SparkMaxConfig();
    conf2.follow(ElevConstants.elev1ID, true);
    elevMtr2.configure(conf2, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    ePIDController = new ProfiledPIDController(ElevConstants.elevP, ElevConstants.elevI, ElevConstants.elevD, new TrapezoidProfile.Constraints(1.75, 0.75));
    
    elevBottomLimit = elevMtr1.getReverseLimitSwitch();
    eLoopController = elevMtr1.getClosedLoopController();
    elevEncoder = elevMtr1.getAbsoluteEncoder();
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

  public void SetGoal( double position) {
    goal = new TrapezoidProfile.State(position, 0.0);
  }

  public void CalculateProfile() {
    elevProfile.calculate(ElevConstants.elevTime, current, goal);
  }

  public void PointMove(double position) {
    if (position < 0.1){
      position = 0.1;
    }
    eLoopController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    current = new TrapezoidProfile.State(elevEncoder.getPosition(), 0.0);
  }
}

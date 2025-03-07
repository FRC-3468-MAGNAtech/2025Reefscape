// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.AbsoluteEncoder;
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevConstants;


public class Evelator extends SubsystemBase {
  private final SparkMax elevMtr1;
  private final SparkMax elevMtr2;
  private final SparkClosedLoopController eLoopController;
  private final ProfiledPIDController ePIDController;
  private final AbsoluteEncoder elevEncoder;
  //private final SoftLimitConfig elevBottomLimit;
  private TrapezoidProfile.State goal;
  private TrapezoidProfile.State current;
  private SoftLimitConfig elevTopLimit;
  private final ElevatorFeedforward feedForward;
  private double position;

  public Evelator() {
    elevMtr1 = new SparkMax(ElevConstants.elev1ID, MotorType.kBrushless);
    elevMtr2 = new SparkMax(ElevConstants.elev2ID, MotorType.kBrushless);
    
    goal = new TrapezoidProfile.State();
    current = new TrapezoidProfile.State();

    elevTopLimit = new SoftLimitConfig().forwardSoftLimit(ElevConstants.tLimit);
    elevTopLimit.forwardSoftLimitEnabled(true);
    //elevBottomLimit = new SoftLimitConfig().reverseSoftLimit(ElevConstants.bLimit);
    //elevBottomLimit.reverseSoftLimitEnabled(true);

    SparkMaxConfig conf = new SparkMaxConfig();
    conf.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    conf.closedLoop.pid(ElevConstants.elevP, ElevConstants.elevI, ElevConstants.elevD);
    conf.limitSwitch.reverseLimitSwitchEnabled(true);
    conf.idleMode(IdleMode.kBrake);
    conf.softLimit.apply(elevTopLimit);
    //conf.softLimit.apply(elevBottomLimit);
    conf.absoluteEncoder.inverted(true);
    elevMtr1.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig conf2 = new SparkMaxConfig();
    conf2.follow(ElevConstants.elev1ID, false);
    elevMtr2.configure(conf2, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    ePIDController = new ProfiledPIDController(ElevConstants.elevP, ElevConstants.elevI, ElevConstants.elevD, new TrapezoidProfile.Constraints(1.75, 0.75));
    
    //elevBottomLimit = elevMtr1.getReverseLimitSwitch();
    eLoopController = elevMtr1.getClosedLoopController();
    elevEncoder = elevMtr1.getAbsoluteEncoder();

    feedForward = new ElevatorFeedforward(0.25, .5, 0);
    
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

  public void elevZero() {
    
  }

  public void elevStay() {
    position = elevEncoder.getPosition();
    eLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward.calculate(position));
  }


  /*
  public void PointMove( double position) {
    ePIDController.setGoal(position);
    elevMtr1.setVoltage(
        ePIDController.calculate(ePIDController.getSetpoint().velocity));
  }
  */
  
  public void pointMove(double position) {
    if (position < 0.1){
      position = 0.1;
    }
    eLoopController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    current = new TrapezoidProfile.State(elevEncoder.getPosition(), 0.0);
    SmartDashboard.putNumber("Elevator position", elevEncoder.getPosition());
  }
}

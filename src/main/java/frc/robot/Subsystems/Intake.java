// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private SparkMax intakeMotor1;
  private SparkMax intakeMotor2;
	private RelativeEncoder intakeEncoder; 
	private PIDController pidcontroller;
	private DigitalInput intakeSensor;
  private SparkLimitSwitch intakeBottomLimit;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor1 = new SparkMax(IntakeConstants.intakeTopID, MotorType.kBrushless);
    intakeMotor2 = new SparkMax(IntakeConstants.intakeBottomID, MotorType.kBrushless);

    SparkMaxConfig conf = new SparkMaxConfig();
    conf.limitSwitch.reverseLimitSwitchEnabled(true);
    conf.idleMode(IdleMode.kBrake);
    conf.smartCurrentLimit(40);
    conf.voltageCompensation(12);

    intakeMotor1.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    intakeMotor2.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    intakeBottomLimit = intakeMotor2.getReverseLimitSwitch();
  }

  public void BallsIn() {
    intakeMotor1.set(-IntakeConstants.intakeIn);
    intakeMotor2.set(IntakeConstants.intakeIn);
  }

  public void BallsOut() {
    intakeMotor1.set(IntakeConstants.intakeOut);
    intakeMotor2.set(-IntakeConstants.intakeOut);
  }

  public void IntakeStop() {
    intakeMotor1.set(0);
    intakeMotor2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

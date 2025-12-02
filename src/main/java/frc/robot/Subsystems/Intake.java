// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase {
  private SparkFlex intakeMotor1;
	// private RelativeEncoder intakeEncoder; 
	// private DigitalInput intakeSensor;
  private DigitalInput sensor;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor1 = new SparkFlex(IntakeConstants.intakeTopID, MotorType.kBrushless);
    sensor = new DigitalInput(IntakeConstants.sensorID);

    SparkFlexConfig conf = new SparkFlexConfig();
    conf.idleMode(IdleMode.kBrake);
    conf.smartCurrentLimit(60);
    conf.voltageCompensation(12);

    intakeMotor1.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void BallsIn() {
    intakeMotor1.set(-IntakeConstants.intakeIn);
  }

  public void BallsOut() {
    intakeMotor1.set(-IntakeConstants.intakeOut);
  }

  public void IntakeStop() {
    intakeMotor1.set(0);
  }

  public boolean getSensor() {
    return !sensor.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("intake", getSensor());
    // This method will be called once per scheduler run
  }
}

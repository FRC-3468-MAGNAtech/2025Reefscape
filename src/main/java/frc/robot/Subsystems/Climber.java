// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  
  private final SparkMax climbMtr;

  public Climber() {
    climbMtr = new SparkMax(ClimberConstants.climbID, MotorType.kBrushless);

    SparkMaxConfig conf = new SparkMaxConfig();
    conf.idleMode(IdleMode.kBrake);
    climbMtr.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void Go() {
    climbMtr.set(ClimberConstants.climbSpeed); 
  }

  public void DontGo() {
    climbMtr.set(ClimberConstants.unClimbSpeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

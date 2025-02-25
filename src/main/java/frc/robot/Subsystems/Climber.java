// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  
  private final SparkMax climbMtr;
  private final SparkClosedLoopController climbLoopController;

  public Climber() {
    climbMtr = new SparkMax(ClimberConstants.climbID, MotorType.kBrushless);

    SparkMaxConfig conf = new SparkMaxConfig();
    conf.idleMode(IdleMode.kBrake);
    conf.closedLoop.pid(ClimberConstants.climbP, ClimberConstants.climbI, ClimberConstants.climbD);
    climbMtr.configure(conf, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    climbLoopController = climbMtr.getClosedLoopController(); 
  }

  public void Go() {
    climbMtr.set(ClimberConstants.climbSpeed); 
  }

  public void GoDown() {
    climbMtr.set(ClimberConstants.unClimbSpeed);
  }

  public void Stop() {
    climbMtr.set(0);
  }

  public void PointMove() {
    climbLoopController.setReference(ClimberConstants.climbPos, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

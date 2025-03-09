// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Evelator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevSetpoints extends Command {
  private final Evelator evelator;
  private double setpoint;
  
  /** Creates a new ElevSetpoints. */
  public ElevSetpoints(Evelator evelator, double height) {
    this.evelator = evelator;
    setpoint = height;
    addRequirements(evelator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    evelator.pointMove(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    evelator.elevStay();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return evelator.isAtSetpoint();
  }
}

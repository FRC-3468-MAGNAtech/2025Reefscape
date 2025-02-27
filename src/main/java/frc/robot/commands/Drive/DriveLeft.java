// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeting;
import frc.robot.Subsystems.LimelightHelpers;
import frc.robot.Subsystems.SwerveSys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveLeft extends Command {
  private SwerveSys swerveSys;

  /** Creates a new DriveLeft. */
  public DriveLeft(SwerveSys sys) {
    swerveSys = sys;
    addRequirements(swerveSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drive = Targeting.alignToReedLeft();
    
    double theta = Math.atan2(drive, 0);
    double r = Math.pow(Math.hypot(drive, 0), 2);

    drive = r * Math.sin(theta);

    swerveSys.drive(1, drive,0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSys.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ( LimelightHelpers.getTA("limelight-right")>= 7 && LimelightHelpers.getTA("limelight-right")<= 7.5) {
      return true;
    } else {
      return false;
    }
  }
}

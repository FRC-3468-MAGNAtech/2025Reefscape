// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.alignment;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeting;
import frc.robot.Subsystems.LimelightHelpers;
import frc.robot.Subsystems.SwerveSys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignLeft extends Command {
  SwerveSys swerveSys;

  /** Creates a new AlignLeft. */
  public AlignLeft(SwerveSys sys) {
    swerveSys = sys;
    addRequirements(swerveSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double straif = Targeting.alignToReedLeft();
    double drive = Targeting.driveToReedLeft();
    
    double theta = Math.atan2(drive, straif);
    double r = Math.pow(Math.hypot(drive, straif), 2);

    straif = r * Math.cos(theta);
    drive = r * Math.sin(theta);

    swerveSys.drive(.4, drive, straif, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSys.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (LimelightHelpers.getTX("limelight-right")<= -1.3 && LimelightHelpers.getTX("limelight-right")>= -3.7 && LimelightHelpers.getTA("limelight-right")>= 7 && LimelightHelpers.getTA("limelight-right")<= 7.5) {
        return true;
    } else {
      return false;
    }
  }
}

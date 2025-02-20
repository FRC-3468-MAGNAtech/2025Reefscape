// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.alignment;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeting;
import frc.robot.Subsystems.LimelightHelpers;
import frc.robot.Subsystems.SwerveSys;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Algae extends Command {
  SwerveSys swerveSys;
  /** Creates a new Algae. */
  public Algae(SwerveSys sys) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveSys = sys;
    addRequirements(swerveSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double straif = Targeting.alignToAlgae();
    double drive = Targeting.driveToAlgae();
    double rot = Targeting.rotToAlgae();
    

    double theta = Math.atan2(drive, straif);
    double r = Math.pow(Math.hypot(drive, straif), 2);


    straif = r * Math.cos(theta);
    drive = r * Math.sin(theta);
    rot = Math.copySign(Math.pow(rot, 2), rot);

    swerveSys.drive(.4, drive, straif, rot, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSys.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (LimelightHelpers.getTA("limelight-forward")<= 80 && LimelightHelpers.getTA("limelight-forward")>= 82) {
        return true;
    } else {
      return false;
    }
  }
}

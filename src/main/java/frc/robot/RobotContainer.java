// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.HIDConstants;
import frc.robot.Subsystems.SwerveSys;
import frc.robot.commands.ArmSetpoints;
import frc.robot.commands.SwerveDrive;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsytems 
  private final Joystick driverController = new Joystick(HIDConstants.driverController);
  private SwerveSys m_SwerveSys = new SwerveSys();
  private final JoystickButton zeroGyro = new JoystickButton(driverController, 11);


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("ThrottleSlider", getThrottle());
  }

  private double getThrottle() {
    double throttle = ((1-driverController.getThrottle())/2.5) + 0.2;
    return throttle;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
		m_SwerveSys.setDefaultCommand(new SwerveDrive(
      () -> getThrottle(),
			() -> MathUtil.applyDeadband(driverController.getY(), HIDConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getX(), HIDConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getZ(), HIDConstants.joystickDeadband),
			true,
			true,
			m_SwerveSys
		));
    zeroGyro.onTrue(new InstantCommand(() -> SwerveSys.resetHeading()));

    l1Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l1;}), new ElevSetpoints(m_Evelator), new InstantCommand(() -> {currentAngle = ArmConstants.l1;}), new ArmSetpoints(m_Arm)));
    l2Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l2;}), new ElevSetpoints(m_Evelator), new InstantCommand(() -> {currentAngle = ArmConstants.l2;}), new ArmSetpoints(m_Arm)));
    l3Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l3;}), new ElevSetpoints(m_Evelator), new InstantCommand(() -> {currentAngle = ArmConstants.l3;}), new ArmSetpoints(m_Arm)));
    l4Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l4;}), new ElevSetpoints(m_Evelator), new InstantCommand(() -> {currentAngle = ArmConstants.l4;}), new ArmSetpoints(m_Arm)));
  
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.HIDConstants;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.SwerveSys;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final Joystick topButtonPad = new Joystick(HIDConstants.topButtonPad);  
  private final Joystick bottomButtonPad = new Joystick(HIDConstants.bottomButtonPad);  

  private SwerveSys m_SwerveSys = new SwerveSys();
  private Intake intake = new Intake();

  private final JoystickButton zeroGyro = new JoystickButton(driverController, 2);

  // Button pad buttons, make constants for the numbers 
  private final JoystickButton algaeOut = new JoystickButton(bottomButtonPad, 2);
  private final JoystickButton algaeIn = new JoystickButton(bottomButtonPad, 7);

	private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    m_SwerveSys.BuilderConfigure();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);

    intake.setDefaultCommand(new IntakeStop(intake));  // maybe fix, hopefully stop

    autoChooser.addOption("2 Back L4 to pro", new PathPlannerAuto("2 Back L4 to pro"));
    autoChooser.addOption("opp. to top right(1), top left(2)", new PathPlannerAuto("opp. to top right(1), top left(2)"));
    autoChooser.addOption("btm. right(1), btm. left(2)", new PathPlannerAuto("btm. right(1), btm. left(2)"));
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

    // Defaults
		m_SwerveSys.setDefaultCommand(new SwerveDrive(
      () -> getThrottle(),
			() -> MathUtil.applyDeadband(driverController.getY(), HIDConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getX(), HIDConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getZ(), HIDConstants.joystickDeadband),
			true,
			true,
			m_SwerveSys
		));

    // Swerve
    zeroGyro.onTrue(new InstantCommand(() -> SwerveSys.resetHeading()));

    // Intake
    algaeIn.onTrue(new IntakeIn(intake));
    algaeOut.onTrue(new IntakeOut(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static Boolean isRedAlliance() {
    return DriverStation.getAlliance().get() == Alliance.Red;
  }
}

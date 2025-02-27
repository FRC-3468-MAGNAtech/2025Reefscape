// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevConstants;
import frc.robot.Constants.HIDConstants;
import frc.robot.Subsystems.Evelator;
import frc.robot.Subsystems.Intake;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Subsystems.SwerveSys;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Arm.ArmSetpoints;
import frc.robot.commands.Drive.DriveLeft;
import frc.robot.commands.Drive.SwerveDrive;
import frc.robot.commands.alignment.Algae;
import frc.robot.commands.alignment.AlignLeft;
import frc.robot.commands.alignment.AlignRight;
import frc.robot.commands.elevator.ElevSetpoints;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private SwerveSys m_SwerveSys = new SwerveSys();
  private Evelator m_Evelator = new Evelator();
  private Intake intake = new Intake();
  public static double currentHeight = 0.1;

  // Buttons
  private final Joystick driverController = new Joystick(HIDConstants.driverController);
  private final Joystick topbuttonPad = new Joystick(HIDConstants.topButtonPad);
  private final Joystick middleButtonPad = new Joystick(HIDConstants.middleButtonPad);
  private final Joystick bottomButtonPad = new Joystick(HIDConstants.bottomButtonPad);
  private final Joystick sideButtonPad = new Joystick(HIDConstants.sideButtonPad);
  private final JoystickButton zeroGyro = new JoystickButton(driverController, 11);
  private final JoystickButton algaeOut = new JoystickButton(bottomButtonPad, 2);
  private final JoystickButton algaeIn = new JoystickButton(bottomButtonPad, 7);

  private final JoystickButton alignLeft = new JoystickButton(topbuttonPad, 4);
  private final JoystickButton alignRight = new JoystickButton(topbuttonPad, 3);
  private final JoystickButton algaefloor = new JoystickButton(bottomButtonPad, 9);

	private final SendableChooser<Command> autoChooser;
  private final JoystickButton elevUp = new JoystickButton(topbuttonPad, 1);
  private final JoystickButton elevDown = new JoystickButton(topbuttonPad, 2);
  private final JoystickButton l1Button = new JoystickButton(bottomButtonPad, 3);
  private final JoystickButton l2Button = new JoystickButton(bottomButtonPad, 4);
  private final JoystickButton l3Button = new JoystickButton(topbuttonPad, 5);
  private final JoystickButton l4Button = new JoystickButton(topbuttonPad, 6);
  private final JoystickButton odometry = new JoystickButton(sideButtonPad, 1);


  public RobotContainer() {
    m_SwerveSys.BuilderConfigure();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);

    intake.setDefaultCommand(new IntakeStop(intake));  // maybe fix, hopefully stop

    NamedCommands.registerCommand("alignLeft", new AlignLeft(m_SwerveSys));
    NamedCommands.registerCommand("alignRight", new AlignRight(m_SwerveSys));
    NamedCommands.registerCommand("zeroGyro", new InstantCommand(() -> SwerveSys.resetHeading()));

    LimeLightConstants.llPIDctrlStraifLeft.setSetpoint(-2);
    LimeLightConstants.llPIDctrlStraifLeft.setTolerance(1);
    LimeLightConstants.llPIDctrlStraifRight.setSetpoint(16);
    LimeLightConstants.llPIDctrlStraifRight.setTolerance(1);

    LimeLightConstants.llPIDctrlDriveLeft.setSetpoint(7);
    LimeLightConstants.llPIDctrlDriveLeft.setTolerance(1);
    LimeLightConstants.llPIDctrlDriveRight.setSetpoint(7);
    LimeLightConstants.llPIDctrlDriveRight.setTolerance(1);

    LimeLightConstants.llPIDctrlAlgaeDrive.setSetpoint(81);
    LimeLightConstants.llPIDctrlAlgaeDrive.setTolerance(1);
    LimeLightConstants.llPIDctrlAlgaeRot.setSetpoint(1);
    LimeLightConstants.llPIDctrlAlgaeRot.setTolerance(1);
    LimeLightConstants.llPIDctrlAlgaeAlign.setSetpoint(23);
    LimeLightConstants.llPIDctrlAlgaeAlign.setTolerance(1);

    autoChooser.addOption("RedM-BkReef(2)-Pro", new PathPlannerAuto("RedM-BkReef(2)-Pro"));
    autoChooser.addOption("BlueT-TopRreef(1)-TopLreef(2)", new PathPlannerAuto("BlueT-TopRreef(1)-TopLreef(2)"));
    autoChooser.addOption("RedT-BtmRreef(1)-BtmLreef(2)", new PathPlannerAuto("RedT-BtmRreef(1)-BtmLreef(2)"));
    autoChooser.addOption("BlueM-BkRF(1)-TopLreef(2)", new PathPlannerAuto("BlueM-BkRF(1)-TopLreef(2)"));
    autoChooser.addOption("Center-FrRf(1)-BtmLreef(2)", new PathPlannerAuto("Center-FrRf(1)-BtmLreef(2)"));
    autoChooser.addOption("BlueM-BkRF(2)-Pro", new PathPlannerAuto("BlueM-BkRF(2)-Pro"));
    autoChooser.addOption("Align", new PathPlannerAuto("Align"));
    autoChooser.addOption("stright`", new PathPlannerAuto("Straight"));
    // Configure the trigger bindings
    //Camera.UpdateLimelight("limelight", m_SwerveSys.odometry, m_SweveSys.imu.getAngularVelocityZDevice().getValueAsDouble());
    configureBindings();
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("ThrottleSlider", getThrottle());
    SmartDashboard.putBoolean("red or blue", isRedAlliance());
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
    elevUp.whileTrue(new frc.robot.commands.elevator.elevUp(m_Evelator));
    elevDown.whileTrue(new frc.robot.commands.elevator.elevDown(m_Evelator));
    l1Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l1;}), new ElevSetpoints(m_Evelator)));
    l2Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l2;}), new ElevSetpoints(m_Evelator)));
    l3Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l3;}), new ElevSetpoints(m_Evelator)));
    l4Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l4;}), new ElevSetpoints(m_Evelator)));
    zeroGyro.onTrue(new InstantCommand(() -> SwerveSys.resetHeading()));

    //l1Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l1;}), new ElevSetpoints(m_Evelator), new InstantCommand(() -> {currentAngle = ArmConstants.l1;}), new ArmSetpoints(m_Arm)));
    //l2Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l2;}), new ElevSetpoints(m_Evelator), new InstantCommand(() -> {currentAngle = ArmConstants.l2;}), new ArmSetpoints(m_Arm)));
    //l3Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l3;}), new ElevSetpoints(m_Evelator), new InstantCommand(() -> {currentAngle = ArmConstants.l3;}), new ArmSetpoints(m_Arm)));
    //l4Button.onTrue(new SequentialCommandGroup(new InstantCommand(() -> {currentHeight = ElevConstants.l4;}), new ElevSetpoints(m_Evelator), new InstantCommand(() -> {currentAngle = ArmConstants.l4;}), new ArmSetpoints(m_Arm)));
  

    //auto movements
    alignLeft.whileTrue(new AlignLeft(m_SwerveSys));
    alignRight.whileTrue(new AlignRight(m_SwerveSys));
    algaefloor.whileTrue(new Algae(m_SwerveSys));

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
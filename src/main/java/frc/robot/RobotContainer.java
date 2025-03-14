// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevConstants;
import frc.robot.Constants.HIDConstants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Evelator;
import frc.robot.Subsystems.Intake;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.Subsystems.SwerveSys;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.alignment.Algae;
import frc.robot.commands.alignment.AlignLeft;
import frc.robot.commands.alignment.AlignRight;
import frc.robot.commands.alignment.L4AlignLeft;
import frc.robot.commands.alignment.L4AlignRight;
import frc.robot.commands.Arm.ArmBackward;
import frc.robot.commands.Arm.ArmForward;
import frc.robot.commands.Arm.ArmSetpoints;
import frc.robot.commands.Arm.ArmStay;
import frc.robot.commands.Arm.ArmStop;
import frc.robot.commands.Climber.ClimbDown;
import frc.robot.commands.Climber.ClimbUp;
import frc.robot.commands.Climber.ClimberStop;
import frc.robot.commands.Drive.DriveLeft;
import frc.robot.commands.Drive.SwerveDrive;
import frc.robot.commands.elevator.elevDown;
import frc.robot.commands.elevator.ElevSetpoints;
import frc.robot.commands.elevator.ElevStay;
import frc.robot.commands.elevator.ElevZero;
import frc.robot.commands.elevator.elevUp;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private Arm m_Arm = new Arm();
  private Climber climber = new Climber();
  private final SendableChooser<Command> autoChooser;
  private boolean reefDirection = true;

  // Controllers
  private final Joystick driverController = new Joystick(HIDConstants.driverController);
  private final Joystick topbuttonPad = new Joystick(HIDConstants.topButtonPad);
  private final Joystick middleButtonPad = new Joystick(HIDConstants.middleButtonPad);
  private final Joystick bottomButtonPad = new Joystick(HIDConstants.bottomButtonPad);
  private final Joystick sideButtonPad = new Joystick(HIDConstants.sideButtonPad);
  
  // Button Pad Buttons
  private final JoystickButton zeroGyro = new JoystickButton(driverController, 11);
    // Intake
  private final JoystickButton algaeOut = new JoystickButton(topbuttonPad, 8);
  private final JoystickButton algaeIn = new JoystickButton(topbuttonPad, 7);
    // Elevator
  private final JoystickButton elevUp = new JoystickButton(topbuttonPad, 5);
  private final JoystickButton elevDown = new JoystickButton(middleButtonPad, 11);
  private final JoystickButton elevZero = new JoystickButton(sideButtonPad, 1);
    // Arm
  private final JoystickButton armForward = new JoystickButton(middleButtonPad, 10);
  private final JoystickButton armbackward = new JoystickButton(topbuttonPad, 4);
  private final JoystickButton cStore = new JoystickButton(topbuttonPad, 2);
  private final JoystickButton aStore = new JoystickButton(middleButtonPad, 8);
    // Limelight
  private final JoystickButton left = new JoystickButton(topbuttonPad, 9);
  private final JoystickButton right = new JoystickButton(topbuttonPad, 10);
  private final JoystickButton algaefloor = new JoystickButton(bottomButtonPad, 9);
    // Climber
  private final JoystickButton ClimbUp = new JoystickButton(middleButtonPad, 1);
  private final JoystickButton ClimbDown = new JoystickButton(bottomButtonPad, 5);
    // Setpoints
  private final JoystickButton aGroundButton = new JoystickButton(middleButtonPad, 5);
  private final JoystickButton cGroundButton = new JoystickButton(middleButtonPad, 4);
  private final JoystickButton l1Button = new JoystickButton(bottomButtonPad, 10);
  private final JoystickButton l2Button = new JoystickButton(middleButtonPad, 6);
  private final JoystickButton l3Button = new JoystickButton(middleButtonPad, 12);
  private final JoystickButton l4Button = new JoystickButton(topbuttonPad, 6);
  private final JoystickButton processorButton = new JoystickButton(middleButtonPad, 7);
  private final JoystickButton humanPlayerButton = new JoystickButton(bottomButtonPad, 4);
  private final JoystickButton netButton = new JoystickButton(topbuttonPad, 1);
  private final JoystickButton topAlg = new JoystickButton(topbuttonPad, 3);
  private final JoystickButton botAlg = new JoystickButton(middleButtonPad, 9);
  private final JoystickButton odometry = new JoystickButton(sideButtonPad, 1);
  private final JoystickButton l4Egt = new JoystickButton(bottomButtonPad, 8);

  private Trigger intakeLimit = new Trigger(() -> intake.limitSwitch());
  private Trigger elevLimit = new Trigger(() -> m_Evelator.limitSwitch());


  public RobotContainer() {
    m_SwerveSys.BuilderConfigure();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoChooser);

    intake.setDefaultCommand(new IntakeStop(intake)); 
    m_Arm.setDefaultCommand(new ArmStay(m_Arm));
    m_Evelator.setDefaultCommand(new ElevStay(m_Evelator));
    climber.setDefaultCommand(new ClimberStop(climber));

  // Start of the commands for AUTO
    NamedCommands.registerCommand("zeroGyro", new InstantCommand(() -> SwerveSys.resetHeading()));

    // Align commands for auto
    NamedCommands.registerCommand("alignLeft", new AlignLeft(m_SwerveSys));
    NamedCommands.registerCommand("alignRight", new AlignRight(m_SwerveSys));
    NamedCommands.registerCommand("l4alignLeft", new L4AlignLeft(m_SwerveSys));
    NamedCommands.registerCommand("l4alignRight", new L4AlignRight(m_SwerveSys));
    NamedCommands.registerCommand("coralStation",new ParallelCommandGroup(
      new ElevSetpoints(m_Evelator, ElevConstants.humanPlayer), 
      new ArmSetpoints(m_Arm, ArmConstants.humanPlayer)));
    
    // Intake commands for auto
    NamedCommands.registerCommand("algaeIN", new IntakeIn(intake));
    NamedCommands.registerCommand("coralIN", new IntakeOut(intake));
    
    // Reef setpoints for auto
    NamedCommands.registerCommand("L1",new ParallelCommandGroup(
      new ElevSetpoints(m_Evelator, ElevConstants.l1), 
      new ArmSetpoints(m_Arm, ArmConstants.l1)));
    NamedCommands.registerCommand("L2",new ParallelCommandGroup(
      new ElevSetpoints(m_Evelator, ElevConstants.l2), 
      new ArmSetpoints(m_Arm, ArmConstants.l2)));
    NamedCommands.registerCommand("L3",new ParallelCommandGroup(
      new ElevSetpoints(m_Evelator, ElevConstants.l3), 
      new ArmSetpoints(m_Arm, ArmConstants.l3)));
    NamedCommands.registerCommand("L4",new ParallelCommandGroup(
      new ElevSetpoints(m_Evelator, ElevConstants.l4), 
      new ArmSetpoints(m_Arm, ArmConstants.l4)));

    // Algae commands for auto
    NamedCommands.registerCommand("Processor",new ParallelCommandGroup(
      new ElevSetpoints(m_Evelator, ElevConstants.processor), 
      new ArmSetpoints(m_Arm, ArmConstants.processor)));
    NamedCommands.registerCommand("Net",new ParallelCommandGroup(
      new ElevSetpoints(m_Evelator, ElevConstants.net), 
      new ArmSetpoints(m_Arm, ArmConstants.net)));
    NamedCommands.registerCommand("store",new ParallelCommandGroup(
      new ElevSetpoints(m_Evelator, ElevConstants.cStore),  
      new ArmSetpoints(m_Arm, ArmConstants.cStore)));
    NamedCommands.registerCommand("human",new ParallelCommandGroup(
      new ElevSetpoints(m_Evelator, ElevConstants.humanPlayer), 
      new ArmSetpoints(m_Arm, ArmConstants.humanPlayer)));
      NamedCommands.registerCommand("algStore", new ParallelCommandGroup(
        new ElevSetpoints(m_Evelator, ElevConstants.aStore),  
        new ArmSetpoints(m_Arm, ArmConstants.net)));
      NamedCommands.registerCommand("topAlg", new ParallelCommandGroup(
        new ElevSetpoints(m_Evelator, ElevConstants.topAlg), 
        new ArmSetpoints(m_Arm, ArmConstants.topAlg)));
  // End for AUTO commands

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
    autoChooser.addOption("BlueM-frontside(1)", new PathPlannerAuto("BlueM-frontside(1)"));
    autoChooser.addOption("Center-leave", new PathPlannerAuto("Center-leave"));
    autoChooser.addOption("BlueM-BkRF(2)-Pro", new PathPlannerAuto("BlueM-BkRF(2)-Pro"));
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
			m_SwerveSys));

    // Swerve
    zeroGyro.onTrue(new InstantCommand(() -> SwerveSys.resetHeading()));

    // Intake
    algaeIn.whileTrue(new IntakeIn(intake));
    algaeOut.onTrue(new IntakeOut(intake));

    // Elevator
    elevUp.whileTrue(new elevUp(m_Evelator));
    elevDown.whileTrue(new elevDown(m_Evelator));
    elevZero.onTrue(new ElevZero(m_Evelator));

    // Arm
    armForward.whileTrue(new ArmForward(m_Arm));
    armbackward.whileTrue(new ArmBackward(m_Arm));
    cStore.onTrue(new ParallelCommandGroup(
      new ElevSetpoints(m_Evelator, ElevConstants.cStore),  
      new ArmSetpoints(m_Arm, ArmConstants.cStore)));
    aStore.onTrue(new ParallelCommandGroup(
      new ElevSetpoints(m_Evelator, ElevConstants.aStore),  
      new ArmSetpoints(m_Arm, ArmConstants.net)));

    zeroGyro.onTrue(new InstantCommand(() -> SwerveSys.resetHeading()));

    // Climber 
    ClimbUp.whileTrue(new ClimbUp(climber));
    ClimbDown.whileTrue(new ClimbDown(climber));
    
    // Setpoints for Elevator
    aGroundButton.onTrue(new ParallelCommandGroup(
        new ElevSetpoints(m_Evelator, ElevConstants.aGround),  
        new ArmSetpoints(m_Arm, ArmConstants.aGround)));
    cGroundButton.onTrue(new ParallelCommandGroup( 
        new ElevSetpoints(m_Evelator, ElevConstants.cGround), 
        new ArmSetpoints(m_Arm, ArmConstants.cGround)));
    humanPlayerButton.onTrue(new ParallelCommandGroup(
        new ElevSetpoints(m_Evelator, ElevConstants.humanPlayer), 
        new ArmSetpoints(m_Arm, ArmConstants.humanPlayer)));

    l1Button.onTrue(new ParallelCommandGroup(new InstantCommand(() -> {if (reefDirection = false) {
          new AlignLeft(m_SwerveSys); 
          }else if (reefDirection = true) {
          new AlignRight(m_SwerveSys);
          }}), 
        new ParallelCommandGroup(
        new ElevSetpoints(m_Evelator, ElevConstants.l1), 
        new ArmSetpoints(m_Arm, ArmConstants.l1))));
    l2Button.onTrue(new ParallelCommandGroup(new InstantCommand(() -> {if (reefDirection = false) {
          new AlignLeft(m_SwerveSys); 
          }else if (reefDirection = true) {
          new AlignRight(m_SwerveSys);
          }}), 
        new ParallelCommandGroup(
        new ElevSetpoints(m_Evelator, ElevConstants.l2),
        new ArmSetpoints(m_Arm, ArmConstants.l2)))); 
    l3Button.onTrue(new ParallelCommandGroup(new InstantCommand(() -> {if (reefDirection = false) {
          new AlignLeft(m_SwerveSys); 
          }else if (reefDirection = true) {
          new AlignRight(m_SwerveSys);
          }}), 
        new ParallelCommandGroup(
        new ElevSetpoints(m_Evelator, ElevConstants.l3), 
        new ArmSetpoints(m_Arm, ArmConstants.l3))));
    l4Button.onTrue(new ParallelCommandGroup(new InstantCommand(() -> {if (reefDirection = false) {
          new L4AlignLeft(m_SwerveSys); 
          }else if (reefDirection = true) {
          new L4AlignRight(m_SwerveSys);
          }}), 
        new SequentialCommandGroup(
        new ParallelCommandGroup(new ElevSetpoints(m_Evelator, ElevConstants.l4), new ArmSetpoints(m_Arm, ArmConstants.l4))
        // new IntakeOut(intake),
        )));

    processorButton.onTrue(new ParallelCommandGroup(
        new ElevSetpoints(m_Evelator, ElevConstants.processor), 
        new ArmSetpoints(m_Arm, ArmConstants.processor)));
    netButton.onTrue(new ParallelCommandGroup(
        new ElevSetpoints(m_Evelator, ElevConstants.net),
        new ArmSetpoints(m_Arm, ArmConstants.net)));

    topAlg.onTrue(new ParallelCommandGroup(
        new ElevSetpoints(m_Evelator, ElevConstants.topAlg), 
        new ArmSetpoints(m_Arm, ArmConstants.topAlg)));
    botAlg.onTrue(new ParallelCommandGroup(
        new ElevSetpoints(m_Evelator, ElevConstants.botAlg), 
        new ArmSetpoints(m_Arm, ArmConstants.botAlg)));

    //auto movements
    left.onTrue(new InstantCommand(() -> reefDirection = false));
    right.onTrue(new InstantCommand(() -> reefDirection = true));
    algaefloor.whileTrue(new Algae(m_SwerveSys));
    l4Egt.whileTrue(new ParallelCommandGroup(new ArmSetpoints(m_Arm, -80), new IntakeOut(intake)));

    // auto store
    // intakeLimit.onTrue(new ParallelCommandGroup(
    //     new ElevSetpoints(m_Evelator, ElevConstants.cStore),
    //     new ArmSetpoints(m_Arm, ArmConstants.cStore)));
    elevLimit.onTrue(new ElevZero(m_Evelator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static boolean isRedAlliance() {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      return true;
    } else {
      return false;
    }
  }
}
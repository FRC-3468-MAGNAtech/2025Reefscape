// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class HIDConstants {
		public static final int driverController = 0;
		public static final int topButtonPad = 1;
		public static final int middleButtonPad = 2;
		public static final int bottomButtonPad = 3;
		public static final int sideButtonPad = 4;

		public static final double joystickDeadband = 0.15;
		// Buttons 
	}

	public static class CANDevices {
		public static final int pigeonId = 2;

		public static final int frontLeftSteerMtrId = 12;
		public static final int frontLeftDriveMtrId =11;
		public static final int frontLeftCanCoderId = 13;

		public static final int frontRightSteerMtrId =21;
		public static final int frontRightDriveMtrId = 22;
		public static final int frontRightCanCoderId = 23;

		public static final int backLeftSteerMtrId = 14;
		public static final int backLeftDriveMtrId = 15;
		public static final int backLeftCanCoderId = 16;

		public static final int backRightSteerMtrId = 25;
		public static final int backRightDriveMtrId = 24;
		public static final int backRightCanCoderId = 26;
	}

	public static class DriveConstants {
		/**
		 * The track width from wheel center to wheel center.
		 */
		public static final double trackWidth = Units.inchesToMeters(24);

		/**
		 * The track length from wheel center to wheel center.
		 */
		public static final double wheelBase = Units.inchesToMeters(24.5);

		/**
		 * The SwerveDriveKinematics used for control and odometry.
		 */
		public static final SwerveDriveKinematics kinematics = 
		new SwerveDriveKinematics(
			new Translation2d(trackWidth / 2.0, wheelBase / 2.0),  // front left
			new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
			new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // back left
			new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // back right
		);
		/**
		* The gear reduction from the drive motor to the wheel.
		* 
		* The drive gear ratios for the different levels can be found from the chart at
		* swervedrivespecialties.com/products/mk41-swerve-module.
		*/
	   	public static final double driveMtrGearReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

	   	/**
		* The gear reduction from the steer motor to the wheel.
		*/
	   	public static final double steerMtrGearReduction = (14.0 / 50.0) * (10.0 / 60.0);
	   
	   	/**
		* Values for our specific MK4i modules
		*/
	   	public static final double wheelRadiusMeters = Units.inchesToMeters(2);
	   	public static final double wheelCircumferenceMeters = 2.0 * wheelRadiusMeters * Math.PI;
	   	public static final double driveBaseRadius = Units.inchesToMeters(14);

	   	public static final double driveMetersPerEncRev = wheelCircumferenceMeters * driveMtrGearReduction;
	   	public static final double driveMetersPerSecPerRPM = driveMetersPerEncRev / 60.0;

	   	public static final double steerRadiansPerEncRev = 2 * Math.PI * DriveConstants.steerMtrGearReduction;
	   	public static final double steerRadiansPerSecPerRPM = steerRadiansPerEncRev / 60;

	   	public static final double kFreeMetersPerSecond = 5820 * driveMetersPerSecPerRPM;

	   	public static final double steerMtrMaxSpeedRadPerSec = 2.0;
	   	public static final double steerMtrMaxAccelRadPerSecSq = 1.0;

	   	public static final double maxDriveSpeedMetersPerSec = 3.5;

		public static final int driveCurrentLimitAmps = 100;

	   	/**
		* The rate the robot will spin with full Rot command.
		*/
		public static final double maxTurnRateRadiansPerSec = 2.0 * Math.PI;
		public static final double drivekP = 0.005;
		public static final double driveI = 0;
		public static final double drivekD = 0.0;

		public static final double steerkP = 1;
		public static final double steerI = 0;
		public static final double steerkD = 0.0;
		
		public static final double ksVolts = 0.667;
		public static final double kvVoltSecsPerMeter = 2.44;
		public static final double kaVoltSecsPerMeterSq = 0.0;

		public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(ksVolts, kvVoltSecsPerMeter, kaVoltSecsPerMeterSq);
		
		// Some wheels would spin backwards
		public static final boolean frontLeftDriveInvert = false;
		public static final boolean frontRightDriveInvert = false;
		public static final boolean backLeftDriveInvert = true;
		public static final boolean backRightDriveInvert = false;

		// This is just-in-case
		public static final boolean frontLeftSteerInvert = true;
		public static final boolean frontRightSteerInvert = true;
		public static final boolean backLeftSteerInvert = true;
		public static final boolean backRightSteerInvert = true;
	}

	public static class ArmConstants {
		public static final int armID = 4;
        public static final double ff = 0.06;

		//feedforward values
		public static final double ffKS = 0.0065;
		public static final double ffKG = .55;
		public static final double ffKV = 0;

		//PID values
		public static final double kP = 0.01;
		public static final double kI = 0;
		public static final double kD = 0.017;

		//speeds for forward and backward
        //negative is forward
		public static final double armBackward = -0.3;
		public static final double armForward = 0.1;

		// Softlimits 
		public static final double forwardSoftLimit = 90;
		public static final double reverseSoftLimit = -45;

		//setpoints for PID
		public static final double cStore = 0;
		public static final double l4Out = -25;
		public static final double aGround = 0;
		public static final double cGround = 83;
		public static final double l1 = 90;
		public static final double l2 = 67;
		public static final double l3 = 67;
		public static final double l4 = 17;
		public static final double tolerance = 2.0;

		//offsets
		public static final double armOffSet = 0;
		public static final double processor = 90;
		public static final double net = 38;
		public static final double humanPlayer = 41;
		public static final double topAlg = 70;
		public static final double botAlg = 70;
	}

	public static class LimeLightConstants {

		public static final double straifRightKP = 0.03;
        public static final double straifLeftKP = 0.03;
		public static final PIDController llPIDctrlStraifLeft = new PIDController(straifLeftKP, 0, 0);
		public static final PIDController llPIDctrlStraifRight = new PIDController(straifRightKP, 0, 0);

        public static final double driveKP = 0.04;
		public static final PIDController llPIDctrlDriveLeft = new PIDController(driveKP, 0,0.01);
		public static final PIDController llPIDctrlDriveRight = new PIDController(driveKP, 0,0);

        public static final double algaeReefKP = 0.01;
        public static final PIDController llPIDctrlAlgaeReef = new PIDController(algaeReefKP, 0, 0);

		public static final double algaeDriveKP = .1;
		public static final double algaeRotKP = .25;
		public static final double algaeAlignKP = .1;
		public static final PIDController llPIDctrlAlgaeDrive = new PIDController(algaeDriveKP, 0, 0);
		public static final PIDController llPIDctrlAlgaeRot = new PIDController(algaeRotKP, 0, 0);
		public static final PIDController llPIDctrlAlgaeAlign = new PIDController(algaeAlignKP, 0, 0);
	}

	public static class ElevConstants {
		// Motor configuration
		public static final int elev1ID = 8;
		public static final int elev2ID = 7;

		//soft limits
		public static final double bLimit = 0.0;
		public static final double tLimit = 38;

		// PID for elevator
		public static final double elevP = .03;
		public static final double elevI = 0;
		public static final double elevD = 1.5;
		public static final double tolerance = 1;

		// Feed Forward
		public static final double ffKS = 0.2;
		public static final double ffKG = 0.4;
		public static final double ffKV = 0;

		public static final double ffKS2 = 0.3;
		public static final double ffKG2 = .6;
		public static final double ffKV2 = 0;

		// trapezoid controller (works with PID to make more smoothe)
		public static final double elevTime = 7.0;

		// movement
		public static final double elevUp = 0.30;
		public static final double elevDown = -0.05;

		// setpoints
		public static final double aGround = 5;
		public static final double cGround = 2;
		public static final double cStore = 0;
		public static final double aStore = 10;
		public static final double l1 = 18;
		public static final double l2 = 14;
		public static final double l3 = 23;
		public static final double l4 = 30.3;
		public static final double processor = 9;
		public static final double net = 39;
		public static final double humanPlayer = 10.5;
		public static final double topAlg = 22;
		public static final double botAlg = 14.1;

	}

	public static class IntakeConstants {
		public static final int intakeTopID = 5;
		public static final double intakeIn = -0.3;   
		public static final double intakeOut = 0.55;
        public static final int sensorID = 9;

	}

	public static class ClimberConstants {

		public static final int climbID = 30;

		// Climb PID
		public static final double climbP = 0.0;
		public static final double climbI = 0.0;
		public static final double climbD = 0.0;

		// Climb movements
		public static final double riseSpeed = 8.5;
		public static final double lowerSpeed = -1;

		// Softlimits 
		public static final double forwardSoftLimit = 290;
		public static final double reverseSoftLimit = -290;

		public static final double climbPos = 0.0;

        //setpoints
        public static final double setPosition = 20;
        public static final double storePositon = 0;
	}

}

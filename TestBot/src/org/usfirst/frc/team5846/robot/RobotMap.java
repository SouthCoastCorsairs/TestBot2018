/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5846.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	
	//Motor Controllers CAN id's
		public static int frontLeft = 0;
		public static int frontRight = 2;
		public static int backLeft = 1;
		public static int backRight = 3;
		
		//Encoders in DIO Ports
		public static int DriveEncoderLeftA = 2;
		public static int DriveEncoderLeftB = 3;
		public static int DriveEncoderRightA = 0;
		public static int DriveEncoderRightB = 1;
		
		public static int liftMotor1 = 0;
		public static int liftMotor2 = 2;
		public static int intakeLeft = 8;
		public static int intakeRight = 9;
		
		public static int Solenoid1 = 0;
		public static int Solenoid2 = 1;
		
		public static double IntakeSpeed = 1;
		public static double LiftSpeed = .5;
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}

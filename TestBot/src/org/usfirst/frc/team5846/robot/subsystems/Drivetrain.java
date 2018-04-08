package org.usfirst.frc.team5846.robot.subsystems;

import org.usfirst.frc.team5846.robot.RobotMap;
import org.usfirst.frc.team5846.robot.commands.Drive;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 *
 */
public class Drivetrain extends Subsystem {
	
	public double RightDistance1;
	public double LeftDistance1;
	
	public Timer timer = new Timer();
	
	private boolean isProfileFinished = false;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	//Talon SRX CAN Motor Controllers
	private WPI_TalonSRX frontLeft = new WPI_TalonSRX(RobotMap.frontLeft);
	private WPI_TalonSRX frontRight = new WPI_TalonSRX(RobotMap.frontRight);
	private WPI_TalonSRX backLeft = new WPI_TalonSRX(RobotMap.backLeft);
	private WPI_TalonSRX backRight = new WPI_TalonSRX(RobotMap.backRight);
		
	public static final double KP = .03;
	public static final double KI = .0015;
	public static final double KD = 0;
	public static final double KF = 0;
		
	public static final double mv = 0;
	public static final double ma = 0;
	public static final double mj = 0;
	public static final double dt = 0;
	
	public static final double KV = 0;
	public static final double KA = 0;
	
	public static final double GP = 0;
	public static final double GD = 0;
		
	public static final double wheel_diameter = 0;
	public static final double wheel_base = 0;
	
	public static double error_prev = 0;
	public static double angle_offset = 0;
	
	
	//Encoders Set Here
	public Encoder LeftEncoder = new Encoder(RobotMap.DriveEncoderLeftA, RobotMap.DriveEncoderLeftB);
	public Encoder RightEncoder = new Encoder(RobotMap.DriveEncoderRightA, RobotMap.DriveEncoderRightB);
	
	//Gyro
	public final AHRS ahrs = new AHRS(SPI.Port.kMXP);


    public void initDefaultCommand() {
    	setDefaultCommand(new Drive());
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public Drivetrain() {
    	
    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
    }
    //All Left Side Drivetrain Motors 
    SpeedControllerGroup Left = new SpeedControllerGroup(frontLeft, backLeft);
    
    //All Right Side Drivetrain Motors
    SpeedControllerGroup Right = new SpeedControllerGroup(frontRight, backRight);
    
    //Primary Drive Method
    public void drive(double turn, double forward) {
    	turn *= .90;
    	//Set with Parameters of Turn and Forward, Which are Set in Drive Command
    	Left.set((-forward) + turn);
    	Right.set(forward + turn);
    }
    
    public void resetEncodersSRX() {
    	frontLeft.setSelectedSensorPosition(0, 0, 0);
    	frontRight.setSelectedSensorPosition(0, 0, 0);
    }
    
    public double getLeftVelocity() {
    	return (frontLeft.getSelectedSensorVelocity(0) * Math.PI * wheel_diameter) / (360) * 10;
    }
    
    public double getRightVelocity() {
    	return (frontRight.getSelectedSensorVelocity(0) * Math.PI * wheel_diameter) / (360) * 10;
    }
    
    public double getLeftAcceleration(double time_prev, double vel_prev) {
    	double deltaTime = timer.get() - time_prev;
    	double deltaVelocity = getLeftVelocity() - vel_prev;
    	
    	
    	return deltaVelocity / deltaTime;
    }
    
    public double getRightAcceleration(double time_prev, double vel_prev) {
    	double deltaTime = timer.get() - time_prev;
    	double deltaVelocity = getRightVelocity() - vel_prev;
    	
    	
    	return deltaVelocity / deltaTime;
    }
    
    //Get Angle Method
    public double getAngle() {
    	return ahrs.getAngle();
    }
    
    
    //Get Distance from Right Encoder in Pulses
    public double getRightDistance() {
    	return RightEncoder.getDistance();
    }
    
    
    //Get Distance from Left Encoder in Pulses
    public double getLeftDistance() {
    	return LeftEncoder.getDistance();
    }
    
    public double RightIN() {
    	return RightDistance1 = (getRightDistance()*18.84)/360; //Conversion from pulses to cm 
    }
    
    
    //Get Distance from Left Encoder in cm
    public double LeftIN() {
    	return LeftDistance1 = (getLeftDistance()*18.84)/360; //Conversion from pulses to cm
    }
    public void ResetEncoders() {
		LeftEncoder.reset();
		RightEncoder.reset();
}
    public double LeftFt() {
    	return (frontLeft.getSelectedSensorPosition(0) * Math.PI * wheel_diameter) / 360;
    }
    
    public double RightFt() {
    	return (frontRight.getSelectedSensorPosition(0) * Math.PI * wheel_diameter) / 360;
    }
    
    public EncoderFollower[] pathSetup(Waypoint[] path) {
    	EncoderFollower left = new EncoderFollower();
    	EncoderFollower right = new EncoderFollower();
    	
    	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 
    			dt, mv, ma, mj);
    	
    	Trajectory toFollow = Pathfinder.generate(path, config);
    	
    	TankModifier modifier = new TankModifier(toFollow).modify(wheel_base);
    	
    	left = new EncoderFollower(modifier.getLeftTrajectory());
    	right = new EncoderFollower(modifier.getRightTrajectory());
    	
    	left.configureEncoder(frontLeft.getSelectedSensorPosition(0), 360, wheel_diameter);
    	right.configureEncoder(frontRight.getSelectedSensorPosition(0), 360, wheel_diameter);
    	
    	left.configurePIDVA(KP, KI, KD, KV, KA);
    	
    	return new EncoderFollower[] {
    			left,
    			right,
    	};
    }
    
    public void resetForPath() {
    	isProfileFinished = false;
    	resetEncodersSRX();
    	ResetGyro();
    }
    
    public void resetPathAngleOffset() {
    	angle_offset = 0;
    }
    
    public boolean getIsProfileFinished() {
    	return isProfileFinished;
    }
    
    public void pathFollow(EncoderFollower[] followers, boolean reverse) {
    	EncoderFollower left = followers[0];
    	EncoderFollower right = followers[1];
    	
    	double l;
    	double r;
    	
    	double localGP = GP;
    	
    	if (!reverse) {
    		localGP *= -1;
    		
    		l = left.calculate(-frontLeft.getSelectedSensorPosition(0));
    		r = right.calculate(-frontRight.getSelectedSensorPosition(0));
    		
    		double heading = reverse ? -getAngle() - 0 : getAngle();
    		
    		double setpoint = Pathfinder.r2d(left.getHeading());
    		SmartDashboard.putNumber("Angle Setpoint", setpoint);
    		
    		double difference = Pathfinder.boundHalfDegrees(setpoint - heading);
    		SmartDashboard.putNumber("Angle Difference", difference);
    		
    		double turn = GP * difference + (GD * ((difference - error_prev) / dt));
    		
    		error_prev = difference;
    		
    		if (!reverse) {
    			tank(l + turn, r - turn);
    		}
    		else {
    			tank(-l + turn, -r - turn);
    		}
    		
    		if (left.isFinished() && right.isFinished()) {
    			isProfileFinished = true;
    			angle_offset = difference;
    		}
    	}
    }


    //For Auto
    //Checks When the Robot is at a Given Distance
    public boolean isAtDistance(double distance) {
    	if(RightIN() >= distance && LeftIN() >= distance) {
    		stopTank();
    		return true;
	}
    	else {
    		return false;
	}
}


    //For Auto
    //Checks When the Robot is at a Given Angle
    public boolean isAtAngle(double angle) {
    	if(Math.abs(getAngle()) >= Math.abs(angle)) {
    		return true;
		
	}
	else {
		return false;
	}
}


    //Reset Gyro Method
    public void ResetGyro() {
    	ahrs.reset();
}


    //For Auto
    //Tank Drive System
    public void tank(double left, double right) {
    	Left.set(left);
    	Right.set(right);
}


    //For Auto
    //Stop Tank Method
    public void stopTank() {
    	this.tank(0, 0);
}
    
    	
    
    
    
}


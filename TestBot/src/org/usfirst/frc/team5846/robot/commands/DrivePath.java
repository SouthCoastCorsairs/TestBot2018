package org.usfirst.frc.team5846.robot.commands;

import org.usfirst.frc.team5846.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;

/**
 *
 */
public class DrivePath extends Command {
	
	Waypoint[] path;
	EncoderFollower[] followers;

    public DrivePath(Waypoint[] path) {
    	requires(Robot.drivetrain);
    	this.path = path;
    	setInterruptible(false);
    	
    	followers = Robot.drivetrain.pathSetup(path);
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.resetForPath();
    	Robot.drivetrain.pathFollow(followers, false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.drivetrain.pathFollow(followers, false);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.drivetrain.getIsProfileFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.stopTank();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	this.end();
    }
}

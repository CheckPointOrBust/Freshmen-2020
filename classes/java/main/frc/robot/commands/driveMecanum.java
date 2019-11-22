/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class driveMecanum extends Command {
  // MecanumDrive drive = new MecanumDrive(Robot.driveTrain.frontLeftMotor, Robot.driveTrain.backLeftMotor, Robot.driveTrain.frontRightMotor, Robot.driveTrain.backRightMotor);
  public driveMecanum() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.driveTrain.setLeftMotorSpeed(0, 0);
    Robot.driveTrain.setRightMotorSpeed(0, 0);
    // drive.setSafetyEnabled(false);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double driverAxisL = Robot.oi.getDriverAxis(RobotMap.ROBOT_DRIVE_YAXIS);
    double driverAxisR = Robot.oi.getDriverAxis(RobotMap.ROBOT_DRIVE_XAXIS_2);
    // turn right
    if (Robot.oi.driver.getTriggerAxis(Hand.kRight) > RobotMap.TRIGGER_DEADZONE) {
      double value = Robot.oi.driver.getTriggerAxis(Hand.kRight) / 2.0;
      Robot.driveTrain.setLeftMotorSpeed(value * -1, 0);
      Robot.driveTrain.setRightMotorSpeed(value, 0);
    } else if (Robot.oi.driver.getTriggerAxis(Hand.kLeft) > RobotMap.TRIGGER_DEADZONE) {
    // turn left
      double value = Robot.oi.driver.getTriggerAxis(Hand.kLeft) / 2.0;
      Robot.driveTrain.setLeftMotorSpeed(value, 0);
      Robot.driveTrain.setRightMotorSpeed(value * -1, 0);
    }
    if ((driverAxisL > RobotMap.DEADZONE && driverAxisR > RobotMap.DEADZONE) || (driverAxisL > (RobotMap.DEADZONE *-1)  && driverAxisR > RobotMap.DEADZONE)) {
			// upper and lower right
      Robot.driveTrain.FLMset(driverAxisR);
      Robot.driveTrain.BRMset(driverAxisR); 
    } else if ((driverAxisL > RobotMap.DEADZONE && driverAxisR < (RobotMap.DEADZONE * -1)) || (driverAxisL > (RobotMap.DEADZONE * -1) && driverAxisR < (RobotMap.DEADZONE * -1))) {
    // upper and lower left
      Robot.driveTrain.BLMset(driverAxisR);
      Robot.driveTrain.FRMset(driverAxisR);
    } else if (driverAxisR > RobotMap.DEADZONE) {
      // drive right
      Robot.driveTrain.FLMset(driverAxisR);
      Robot.driveTrain.BRMset(driverAxisR);
      Robot.driveTrain.FRMset(driverAxisR * -1);
      Robot.driveTrain.BLMset(driverAxisR * -1);
    } else if (driverAxisR < (RobotMap.DEADZONE * -1)) {
      // drive left
      Robot.driveTrain.FLMset(driverAxisR * -1);
      Robot.driveTrain.BRMset(driverAxisR * -1);
      Robot.driveTrain.FRMset(driverAxisR);
      Robot.driveTrain.BLMset(driverAxisR);
    } else if (driverAxisL > RobotMap.DEADZONE){
      // drive up
      Robot.driveTrain.setLeftMotorSpeed(driverAxisL, 0);
      Robot.driveTrain.setRightMotorSpeed(driverAxisL, 0);
    } else if (driverAxisL < (RobotMap.DEADZONE * -1)) {
      // drive down
      Robot.driveTrain.setLeftMotorSpeed(driverAxisL, 0);
      Robot.driveTrain.setRightMotorSpeed(driverAxisL, 0);
    } else {
      // turn off everything. don't kill me please :+)
      Robot.driveTrain.setLeftMotorSpeed(0, 0);
      Robot.driveTrain.setRightMotorSpeed(0, 0);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    initialize();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

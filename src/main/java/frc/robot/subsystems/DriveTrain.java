/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  // Declartion of driver motors (hi)
  private TalonSRX frontRightMotor = new TalonSRX(RobotMap.FRONT_RIGHT_MOTOR);
  private TalonSRX backRightMotor = new TalonSRX(RobotMap.BACK_RIGHT_MOTOR);
  private TalonSRX frontLeftMotor = new TalonSRX(RobotMap.FRONT_LEFT_MOTOR);
  private TalonSRX backLeftMotor = new TalonSRX(RobotMap.BACK_LEFT_MOTOR);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    backLeftMotor.follow(frontRightMotor);
    backRightMotor.follow(frontLeftMotor);
    backLeftMotor.setInverted(true);
    frontLeftMotor.setInverted(true);
  }

  public void setRightMotorSpeed(double speedY, double speedX) {
    if (getDeadzone(speedY)) {
      speedY = 0;
    }
    if (getDeadzone(speedX)) {
      speedX = 0;
    }
      frontRightMotor.set(ControlMode.PercentOutput, speedY + speedX);
      backLeftMotor.set(ControlMode.PercentOutput, speedY + speedX);
  }

  public void setLeftMotorSpeed(double speedY, double speedX) {
    if (getDeadzone(speedY)) {
      speedY = 0;
    }
    if (getDeadzone(speedX)) {
      speedX = 0;
    }
    frontLeftMotor.set(ControlMode.PercentOutput, speedY + speedX);
    backLeftMotor.set(ControlMode.PercentOutput, speedY + speedX);
  }

  public void zeroOut() {
    frontLeftMotor.set(ControlMode.PercentOutput, 0);
    frontRightMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean getDeadzone(double speed) {
    return Math.abs(speed) < RobotMap.DEADZONE;
  }


}

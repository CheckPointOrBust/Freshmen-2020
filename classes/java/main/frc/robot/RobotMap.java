/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // Motor Port Number
  // public static final int FRONT_RIGHT_MOTOR = 5;
  // public static final int BACK_RIGHT_MOTOR = 6;
  // public static final int FRONT_LEFT_MOTOR = 1;
  // public static final int BACK_LEFT_MOTOR = 12;

  public static final int FRONT_RIGHT_MOTOR = 3;
  public static final int BACK_RIGHT_MOTOR = 2;
  public static final int FRONT_LEFT_MOTOR = 10;
  public static final int BACK_LEFT_MOTOR = 9;

  // Drive Controller Port
  public static final int ROBOT_DRIVE_CONTROLLER = 0;
  public static final int ROBOT_DRIVE_YAXIS = 1;
  public static final int ROBOT_DRIVE_XAXIS = 0;
  public static final int Y_AXIS = 5;
  public static final int ROBOT_DRIVE_XAXIS_2 = 4;

  // dead zone
  public static final double DEADZONE = 0.3; //why not
  public static final double TRIGGER_DEADZONE = 0.1;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}

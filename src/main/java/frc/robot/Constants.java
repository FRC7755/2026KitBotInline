// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 2;
    public static final int LEFT_FOLLOWER_ID = 1;
    public static final int RIGHT_LEADER_ID = 4;
    public static final int RIGHT_FOLLOWER_ID = 3;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    // Define physical constants (use meters and radians)
    public static final double CAMERA_HEIGHT_METERS = 0.2; // Height of lens from floor
    public static final double TARGET_HEIGHT_METERS = 1.124; // Height of AprilTag center from floor
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(22.5); // Mounting angle
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 6;
    public static final int LAUNCHER_MOTOR_ID = 5;
    public static final int INTAKE_MOTOR_ID = 8;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;
    public static final int INTAKE_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 7;//10
    public static final double LAUNCHING_FEEDER_VOLTAGE = 10;//9
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = .5;

    public static final double INTAKE_RPMS =3500;
    public static final double FEEDER_RPMS = 3500;
    public static final double LAUNCHER_RPMS = 3200;
    public static final double LAUNCHER_SLOW_RPMS = -600;
    public static final double RPM_TOLERANCE = 50;
    public static final double KNOB_ZERO = 0;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int BOX_CONTROLLER_PORT = 1;
    public static final int KNOB_CONTROLLER_PORT = 2;
    

    // This value is multiplied by the joystick value when driving the robot to
    // help avoid driving and turning too fast and being difficult to control
    public static final double DRIVE_SCALING = 1;
    public static final double ROTATION_SCALING = 1;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 7;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 45;
    public static final double CLIMBER_VOLTAGE = 3;
  }
}

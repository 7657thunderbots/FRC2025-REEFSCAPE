// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import com.pathplanner.lib.config.PIDConstants;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // CONSTANT MEMBERS

  // public static final CommandXboxController driverController =
  //         new CommandXboxController(DriveteamConstants.kDriverControllerPort);
  // public static final CommandXboxController operatorController =
  //         new CommandXboxController(DriveteamConstants.kOperatorControllerPort);
  
  // public static final double ROBOT_MASS = (100) * 0.453592; // 32lbs * kg per pound***Need to change
  // // public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  // public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  
  // CONTROL CONSTANT CLASSES

  public static class Vision {
      public static final String kCameraName = "HD_Webcam_C615";
      // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
      public static final Transform3d kRobotToCam =
              new Transform3d(new Translation3d(0.127, .3620516, .2397125), new Rotation3d(0, 0, 0));

      // The layout of the AprilTags on the field
      public static final AprilTagFieldLayout kTagLayout =
              AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

      // The standard deviations of our vision estimated poses, which affect correction rate
      // (Fake values. Experiment and determine estimation noise on an actual robot.)
      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class AutonConstants
  {

    public static  PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0);
    public static  PIDConstants ANGLE_PID   = new PIDConstants(5, 0, 0.0);
  }

  public static class DriveteamConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kOperatorControllerPort = 1;

  }

  // public static class OperatorConstants
  // {

  //   // Joystick Deadband
  //   public static final double LEFT_X_DEADBAND  = 0.1;
  //   public static final double LEFT_Y_DEADBAND  = 0.1;
  //   public static final double RIGHT_X_DEADBAND = 0.1;
  //   public static final double TURN_CONSTANT    = 6;
  // }

  // SUBSYSTEM CONSTANT CLASSES

  // public static class ClimberConstants {
  //   public static final int leftClimberMotorID = 26;
  //   public static final int rightClimberMotorID = 27;
  //   public static final int lowerID = 5;

  // }
  // public static class FeederConstants {
  //   public static final int feederMotorID = 28;

  //   public static final int beamBrakeChannel = 3;

  // }

  // public static class IntakeConstants {
  //   public static final int intakeMotorLeftID = 24;
  //   public static final int intakeMotorRightID = 23;
  //   public static final int centerMotorID = 25;
  //   public static final int forwardChannelPort = 1;
  //   public static final int reverseChannelPort = 0;

  // }
  // public static class ShooterConstants {
  //   public static final int leftShooterID = 20;
  //   public static final int rightShooterID = 21;

  //   public static final int forwardChannelPort = 2;
  //   public static final int reverseChannelPort = 3;

  // }

  // public static class LEDConstants {
  //   public static final int port = 36;
  //   public static final int length = 27;
  // }

  // public static class ElevatorConstants {

  //   public static final int leftLiftID = 32;
  //   public static final int rightLiftID = 33;

  //   public static final int leftLimitSwitchTop = 6;
  //   public static final int leftLimitSwitchBottom = 5;

  //  public static final int rightLimitSwitchTop = 8;
  //  public static final int rightLimitSwitchBottom = 7;

  // }
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(2);//14.5
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  // public static final class DrivebaseConstants
  // {

  //   // Hold time on motor brakes when disabled
  //   public static final double WHEEL_LOCK_TIME = 10; // seconds
  // }

}
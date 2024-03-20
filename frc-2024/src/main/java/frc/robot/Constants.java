// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final double kNeoTicksPerRevolution = 42;
  public static final double kFalconTicksPerRevolution = 2048;
  public static final int kJoystick2ID = 1;

  public static class DriveConstants {
    // public static final double kDriveEncoderPositionToMeters = (1.0 / 6.75) * Units.inchesToMeters(4.0 * Math.PI); 
    public static final double kDriveEncoderPositionToMeters = Units.inchesToMeters(4.0 * Math.PI) / 6.75;
    public static final double kDriveEncoderVelocityToMetersPerSec = kDriveEncoderPositionToMeters / 60.0;
    public static final double kTurnEncoderPositionToRadians = 2.0 * Math.PI;
    public static final double kTranslationalDeadbandMetersPerSecond = 0.001;
    public static final double  kMaxTranslationalMetersPerSecond = Units.feetToMeters(15.1);
    // public static final double kMaxTranslationalMetersPerSecond = Units.feetToMeters(4.5);

    
    public static final double kPTurning = 0.5;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0;
    public static final double kPDriving = 0.4;
    public static final double kIDriving = 0.0;
    public static final double kDDriving = 0.0;
    public static final double kWheelBase = Units.inchesToMeters(28);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(0.3556 - 0.065, 0.3556 - 0.068),
      new Translation2d(0.3556 - 0.066, -0.3556 + 0.066),
      new Translation2d(0.3556 - 0.645, -0.3556 + 0.644),
      new Translation2d(0.3556 - 0.644, -0.3556 + 0.063)
    );
    // TODO: gather all of these constants.
    public static int kFrontLeftDriveId = 2;
    public static int kFrontLeftTurnId = 1;
    public static int kFrontLeftAbsoluteEncoderPort = 9;
    public static double kFrontLeftAbsoluteEncoderOffset = -0.529990463821544 - (Math.PI / 2.0);
    public static boolean kFrontLeftDriveReversed = false;

    public static int kFrontRightDriveId = 4;
    public static int kFrontRightTurnId = 3;
    public static int kFrontRightAbsoluteEncoderPort = 7;
    public static double kFrontRightAbsoluteEncoderOffset = 1.415447275814819 + (Math.PI / 2.0);
    public static boolean kFrontRightDriveReversed = true;

    public static int kBackLeftDriveId = 8;
    public static int kBackLeftTurnId = 7;
    public static int kBackLeftAbsoluteEncoderPort = 8;
    public static double kBackLeftAbsoluteEncoderOffset = 1.253910661599266 + (Math.PI / 2.0);
    public static boolean kBackLeftDriveReversed = false;

    public static int kBackRightDriveId = 6;
    public static int kBackRightTurnId = 5;
    public static int kBackRightAbsoluteEncoderPort = 6;
    public static double kBackRightAbsoluteEncoderOffset = -0.456511594330128 - (Math.PI / 2.0);
    public static boolean kBackRightDriveReversed = true;

    public static double kDeadband = 0.05;
    public static double kTeleopMaxAccelMetersPerSecondSquared = 2.5;
    // public static double kTeleopMaxAccelMetersPerSecondSquared = 0.5;
    public static double kTeleopMaxAngularAccelRadiansPerSecondSquared = 3;
    // public static double kTeleopMaxAngularAccelRadiansPerSecondSquared = 0.5;
    // public static double kTeleopMaxSpeedMetersPerSecond = kMaxTranslationalMetersPerSecond/4.0;
    public static double kTeleopMaxSpeedMetersPerSecond = kMaxTranslationalMetersPerSecond;
    // public static double kTeleopMaxTurningRadiansPerSecond = 4.0 * Math.PI;
    public static double kTeleopMaxTurningRadiansPerSecond = 1.5 * Math.PI;
    public static int kDriveJoystickId = 0;
    public static int kJoystickXAxis = 1;
    public static int kJoystickYxis = 0;
    public static int kJoystickRotAxis = 4;
    public static boolean kFrontLeftTurningReversed = true;
    public static boolean kFrontRightTurningReversed = true;
    public static boolean kBackLeftTurningReversed = true;
    public static boolean kBackRightTurningReversed = false;

    public static double kP_X = 0.75;
    public static double kI_X = 0;
    public static double kD_X = 0;

    public static double kP_Y = 0.75;
    public static double kI_Y = 0;
    public static double kD_Y = 0;

    public static double kP_Theta = 0.75;
    public static double kI_Theta = 0;
    public static double kD_Theta = 0;

  }

  public final static class PivotConstants {
    public static final int kMotorID1 = 1;
    public static final int kMotorID2 = 2;
    public static final int kMotorID3 = 3;
    public static final int kMotorID4 = 4;
    public static final int kEncoderID = 3;
    public static final int kForwardSwitchID = 4;
    public static final int kBackwardSwitchID = 1;
    public static final double kGearRatio = 182.0444;
    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kDeadBand = 0.05;
    public static final double kAbsEncoderOffset = 0.8624; // subtractive.
  }

  public final static class WristConstants {
    public static final int kMotorID = 5;
    public static final int kEncoderID = 0;
    public static final int kForwardSwitchID = 12;
    public static final int kBackwardSwitchID = 19;
    public static final double kGearRatio = 51.9273;
    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kDeadBand = 0.05;
    public static final double kEncoderOffset = 0.886434;
  }

  public static final class ElevatorConstants {
    public static final int kMotorID1 = 12;
    public static final int kMotorID2= 13;
    public static final int kEncoderID = 2;
    public static final int kTopSwitchID = 13;
    public static final int kBottomSwitchID = 10;
    public static final double RotationsPerInch = 0.00000001;
    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorID = 9;
  }

  public static final class ShooterConstants {
    public static final int kShooterMotorID1 = 10;
    public static final int kShooterMotorID2 = 11;
  }
}
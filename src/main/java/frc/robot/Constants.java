// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DeviceIds {
    public static final int driver1Port = 0;
    
    public static final int fLSwerveDrive = 1;
    public static final int fLSwerveTurn = 2;
    public static final int fRSwerveDrive = 7;
    public static final int fRSwerveTurn = 8;
    public static final int bLSwerveDrive = 3;
    public static final int bLSwerveTurn = 4;
    public static final int bRSwerveDrive = 5;
    public static final int bRSwerveTurn = 6;

    public static final int fLEncoder = 9;
    public static final int fREncoder = 10;
    public static final int bLEncoder = 11;
    public static final int bREncoder = 12;

    public static final int leftElevator = 9;
    public static final int rightElevator = 10;

    public static final int algaeBase = 11;
    public static final int algaeWheel = 12;

    public static final int leftGrabber = 13;
    public static final int rightGrabber = 14;
    public static final int tilt = 15;

    /** DIO port on rio */
    public static final int armLimitSwitch = 0;
  }

  public static class DrivetrainConstants {
    /** measured in meters per second */
    public static double maxVelocity = 1.5;
    /** meters per second^2 */
    public static double maxAcceleration = 0.1;
    /** R=rotations per second */
    public static double maxAngularVelocity = 3;

    public static double maxPIDError = 0;

    public static double angleKP = 4.0;
    public static double angleKI = 0.0;
    public static double angleKD = 0.0;

    public static double driveKP = 0.0;
    public static double driveKI = 0.0;
    public static double driveKD = 0.0;

    public static double driveKS = 0.1615;
    public static double driveKV = 1.2;
    public static double driveKA = 0.3;

    /** Center of one wheel to another */
    public static final double wheelBaseLength = Units.inchesToMeters(25);
    /** Radius of the wheel in meters */
    public static final double wheelRadius = Units.inchesToMeters(2);

    public static double frontLeftEncoderOffset = Math.toRadians(-89.82) - Math.PI / 2.0;
    public static double frontRightEncoderOffset = Math.toRadians(-180.703) + Math.PI / 2.0;
    public static double backLeftEncoderOffset = Math.toRadians(-80.069-44.75) - Math.PI / 2.0;
    public static double backRightEncoderOffset = Math.toRadians(-88.330) + Math.PI / 2.0;

    public static double positionMultiplier = 1;
     
    public static Vector<N3> stateStandardDeviation = VecBuilder.fill(0.01, 0.01, 0.02);
    public static Vector<N3> visionStandardDeviation = VecBuilder.fill(0.5, 0.5, 1.0);
    public static double periodTime = 0.02;

  }

  public static class AlgaeIntakeConstants {
    public static double startIntakeAngle = 0.0;
    public static double finalIntakeAngle = Math.PI/4;

    public static double intakeVoltage = 3;
    public static double scoreVoltage = 5;
  }

  public static class ArmConstants {
    public static double intakeVoltage = 4.5;
    public static double outakeVoltage = 4.5;
  }

  public static class TiltConstants {
    public static double positionOffset = 0;
    public static double intakePostition = 0-positionOffset;
    public static double l4Score = 3*Math.PI/4;
  }

  public static class ElevatorConstants {
    /** circumference of the gear in meters */
    public static double positionConversionFactor = 2*Math.PI*2;
    /** meters */
    public static double pickupPosition = 0;
    
    public static double l4Score = 0;
    public static double l3Score = 0;
    public static double l2Score = 0;
    public static double troughScore = 0;
  }
}

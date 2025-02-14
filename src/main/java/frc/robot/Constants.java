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
    public static int driver1Port = 0;
    
    public static int fLSwerveDrive = 1;
    public static int fLSwerveTurn = 2;
    public static int fRSwerveDrive = 3;
    public static int fRSwerveTurn = 4;
    public static int bLSwerveDrive = 5;
    public static int bLSwerveTurn = 6;
    public static int bRSwerveDrive = 7;
    public static int bRSwerveTurn = 8;

    public static int leftElevator = 9;
    public static int rightElevator = 10;

    public static int algaeBase = 11;
    public static int algaeWheel = 12;

    public static int leftGrabber = 13;
    public static int rightGrabber = 14;
    public static int tilt = 15;
    
  }

  public static class DrivetrainConstants {
    public static double maxVelocity = 3.0; //meters per second
    public static double maxAcceleration = 0.1; //meters per second^2
    public static double maxAngularVelocity = Math.PI; //0.5 rotations per second
    public static double maxAngularAcceleration = Math.PI/8; //rotations per second^2

    public static double maxPIDError = 0;

    public static double kP = 8.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static final double wheelBaseLength = Units.inchesToMeters(25); //Center of one wheel to another
    public static final double wheelRadius = Units.inchesToMeters(2); //Radius of the wheel in meters

    public static double frontLeftEncoderOffset = Math.toRadians(-89.82) - Math.PI / 2.0;
    public static double frontRightEncoderOffset = Math.toRadians(-180.703) + Math.PI / 2.0;
    public static double backLeftEncoderOffset = Math.toRadians(-80.069-44.75) - Math.PI / 2.0;
    public static double backRightEncoderOffset = Math.toRadians(-88.330) + Math.PI / 2.0;

    public static double positionMultiplier = 1;
     
    public static Vector<N3> stateStandardDeviation = VecBuilder.fill(0.01, 0.01, 0.02);
    public static Vector<N3> visionStandardDeviation = VecBuilder.fill(0.5, 0.5, 1.0);
    public static double periodTime = 0;

  }

  public static class AlgaeIntakeConstants {
    public static double startIntakeAngle = 0.0;
    public static double finalIntakeAngle = Math.PI/4;

    public static double intakeVoltage = 3;
    public static double scoreVoltage = 5;

  }

  public static class ArmConstants {
    public static double intakeVoltage = 4.5;
    public static double outakeVoltage = 5;

  }


}

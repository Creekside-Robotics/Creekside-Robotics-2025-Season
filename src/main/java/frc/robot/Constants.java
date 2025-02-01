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
    
    public static int fLSwerveDrive = 0;
    public static int fLSwerveTurn = 0;
    public static int fRSwerveDrive = 0;
    public static int fRSwerveTurn = 0;
    public static int bLSwerveDrive = 0;
    public static int bLSwerveTurn = 0;
    public static int bRSwerveDrive = 0;
    public static int bRSwerveTurn = 0;

    public static int algaeBase = 0;
    public static int algaeWheel = 0;
    
  }

  public static class DrivetrainConstants {
    public static double maxVelocity = 3.0; //meters per second
    public static double maxAngularVelocity = Math.PI; //0.5 rotations per second

    public static final double wheelBaseLength = Units.inchesToMeters(25); //Center of one wheel to another

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
    public static double finalIntakeAngle = Units.degreesToRadians(45);

  }


}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightResults;

public class Drivetrain extends SubsystemBase {
  double baseLength = DrivetrainConstants.wheelBaseLength;

  //? i think this should just be put in kinematics directly without declaring new variables, but who cares i guess (the garbage collector cares i guess)
  private final Translation2d frontLeftPosition = new Translation2d(baseLength/2, baseLength/2);
  private final Translation2d frontRightPosition = new Translation2d(baseLength/2, -baseLength/2);
  private final Translation2d backLeftPosition = new Translation2d(-baseLength/2, baseLength/2);
  private final Translation2d backRightPosition = new Translation2d(-baseLength/2, -baseLength/2);

  final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeftPosition,
    frontRightPosition,
    backLeftPosition,
    backRightPosition
  );

  private final SwerveModule m_frontLeft = new SwerveModule(DeviceIds.fLSwerveDrive, DeviceIds.fLSwerveTurn, DeviceIds.fLEncoder);
  private final SwerveModule m_frontRight = new SwerveModule(DeviceIds.fRSwerveDrive, DeviceIds.fRSwerveTurn, DeviceIds.fREncoder);
  private final SwerveModule m_backLeft = new SwerveModule(DeviceIds.bLSwerveDrive, DeviceIds.bLSwerveTurn, DeviceIds.bLEncoder);
  private final SwerveModule m_backRight = new SwerveModule(DeviceIds.bRSwerveDrive, DeviceIds.bRSwerveTurn, DeviceIds.bREncoder);

  private final ADIS16448_IMU gyro = new ADIS16448_IMU();

  private final SwerveDrivePoseEstimator poseEstimator;

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          kinematics,
          new Rotation2d(gyro.getGyroAngleZ()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  private final Field2d field2d = new Field2d();

  public Drivetrain() {
    gyro.reset();

    poseEstimator = new SwerveDrivePoseEstimator(kinematics, 
      getGyroAngle(), 
      getModulePositions(), 
      new Pose2d());

    SmartDashboard.putData("Field Display", field2d);
  }

  @Override
  public void periodic() {
    this.poseEstimator.update(getGyroAngle(), getModulePositions());
    this.updatePoseWithLimelight();
    // SmartDashboard.putNumber("Current Rot:", m_frontLeft.getTurnPosRadians());
    // SmartDashboard.putNumber("Desired Rot:", Math.toRadians(45));
    // double volts = m_frontLeft.turnController.calculate(m_frontLeft.getTurnPosRadians(), Math.toRadians(45));
    // SmartDashboard.putNumber("Voltage:", volts);
    // SmartDashboard.putNumber("Error:", m_frontLeft.turnController.getError());

    // DrivetrainConstants.angleKP = SmartDashboard.getNumber("p", 0);
    // DrivetrainConstants.angleKI = SmartDashboard.getNumber("i", 0);
    // DrivetrainConstants.angleKD = SmartDashboard.getNumber("d", 0);
  }

  public void setModuleStates(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModule[] modules = {this.m_frontLeft, this.m_frontRight, this.m_backLeft, this.m_backRight};

    SwerveModuleState[] states =
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize( 
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroAngle()) : new ChassisSpeeds(xSpeed, ySpeed, rot), DrivetrainConstants.periodTime));

    modules[0].setDesiredState(states[0], true);
    modules[1].setDesiredState(states[1], false);
    modules[2].setDesiredState(states[2], false);
    modules[3].setDesiredState(states[3], false);
  }

  public void updatePoseWithLimelight() {
    LimelightResults results = LimelightHelpers.getLatestResults("limelight");
    if (results.targetingResults.valid && results.targetingResults.getBotPose2d_wpiBlue().getX() != 0) {
      Pose2d pose = results.targetingResults.getBotPose2d_wpiBlue();

      double distance = results.targetingResults.botpose_avgdist;
      double timestamp = results.targetingResults.timestamp_RIOFPGA_capture;

      SmartDashboard.putNumber("Timestamp", timestamp);
      displayDrivetrainPose(pose);

      this.poseEstimator.addVisionMeasurement(pose, timestamp, DrivetrainConstants.visionStandardDeviation.times(distance));
    }
  }

  private void displayDrivetrainPose(Pose2d pose) {
    this.field2d.setRobotPose(pose);
    
    SmartDashboard.putData("Field Display", field2d);
  }

  private Rotation2d getGyroAngle() {
    Rotation2d rotation = Rotation2d.fromDegrees(-this.gyro.getGyroAngleZ());
    SmartDashboard.putNumber("Gyro", rotation.getRadians());
    return rotation;
  }

  /** returns module positions as list of SMP's (position, angle) */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModule[] modules = {this.m_frontLeft, this.m_frontRight, this.m_backLeft, this.m_backRight};
    SwerveModulePosition[] positions = {null, null, null, null};

    for (int i = 0; i < modules.length; i++) {
      positions[i] = new SwerveModulePosition(
        modules[i].getPosition().distanceMeters * DrivetrainConstants.positionMultiplier,
        modules[i].getPosition().angle
      );
    }
    return positions;
  }

  /** updates Odometry using gyro and swerve positions */
  public void updateOdometry() {
    m_odometry.update(
        getGyroAngle(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  // ----------- COMMANDS -----------
  public Command commandResetGyro() {
    return this.runOnce(() -> this.gyro.reset());
  }

  public Command forwardAuto(){
    return this.run(() -> this.setModuleStates(1, 0, 0, false));
  }
}

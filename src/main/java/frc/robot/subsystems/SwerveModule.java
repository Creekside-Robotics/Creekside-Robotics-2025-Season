package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;



public class SwerveModule {
  private static int kEncoderResolution = 4096;

  //Motor controllers and encoders
  private static SparkMax driveMotor;
  private static SparkMax turnMotor;
  
  AbsoluteEncoder driveEncoder;
  AbsoluteEncoder turnEncoder;

  SparkClosedLoopController drivePID;
  SparkClosedLoopController turnPID;

  /* Constructs a new Swerve module with:
   * Drive motor
   * Turn Motor
   * Config Both
   */
  public SwerveModule(int driveMotorID, int turningMotorID) {
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getAbsoluteEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    drivePID = driveMotor.getClosedLoopController();
    turnPID = turnMotor.getClosedLoopController();

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig turnConfig = new SparkMaxConfig();

    //IdleMode Config
    driveConfig.idleMode(IdleMode.kBrake);

    //Position and Velocity Conversion Factors
    driveConfig.encoder.positionConversionFactor(1/kEncoderResolution); //Rotation per encoder counts
    driveConfig.encoder.velocityConversionFactor(1); //Counts per second

    //Set feedback sensor and pid constants
    driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    driveConfig.closedLoop.pid(Constants.DrivetrainConstants.kP,Constants.DrivetrainConstants.kI,Constants.DrivetrainConstants.kD);

    driveConfig.closedLoop.maxMotion.maxVelocity(Constants.DrivetrainConstants.maxVelocity);
    driveConfig.closedLoop.maxMotion.maxAcceleration(Constants.DrivetrainConstants.maxAcceleration);
    driveConfig.closedLoop.maxMotion.allowedClosedLoopError(Constants.DrivetrainConstants.maxPIDError);

    driveConfig.signals.primaryEncoderPositionPeriodMs(5);
    
    turnConfig.apply(driveConfig);

    turnConfig.closedLoop.maxMotion.maxVelocity(Constants.DrivetrainConstants.maxAngularVelocity);
    turnConfig.closedLoop.maxMotion.maxAcceleration(Constants.DrivetrainConstants.maxAngularAcceleration);
    turnConfig.closedLoop.maxMotion.allowedClosedLoopError(Constants.DrivetrainConstants.maxPIDError);
    
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity()*driveMotor.configAccessor.encoder.getVelocityConversionFactor(), new Rotation2d(turnEncoder.getPosition()));
  }

  public double toRPM(double velocityMPS){
    return (velocityMPS * 60)/(2*Math.PI*Constants.DrivetrainConstants.wheelRadius);
  }

  //returns current position of the module
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition()));
  }

  //sets desired state of the module
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d encoderRotation = new Rotation2d(turnEncoder.getPosition()); //current rotation

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Set the reference of the drive motor based on desired state
    drivePID.setReference(toRPM(desiredState.speedMetersPerSecond), ControlType.kMAXMotionVelocityControl);

    // Set the reference of the turn motor based on desired state
    turnPID.setReference(desiredState.angle.getRotations(), ControlType.kMAXMotionPositionControl);
    
  
  }
}
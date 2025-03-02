// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class SwerveModule extends SubsystemBase{
  //Motor controllers and encoders
  private SparkMax driveMotor;
  private SparkMax turnMotor;
  
  RelativeEncoder driveEncoder;
  CANcoder turnEncoder;

  // PIDController turnController;
  PIDController turnController;

  PIDController driveController;
  SimpleMotorFeedforward driveFeedforward;
  // SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.5, kEncoderResolution)

  /** 
   * Constructs a new Swerve module with drive and turn motor and applies respective configuration to both.
   * @param driveMotorID ID for the drive motor
   * @param turningMotorID Id for the turn motor
   * @param canCoderID ID for CANcoder
   */
  public SwerveModule(int driveMotorID, int turningMotorID, int canCoderID) {
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = new CANcoder(canCoderID);
    
    turnController = new PIDController(DrivetrainConstants.angleKP, DrivetrainConstants.angleKI, DrivetrainConstants.angleKD);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
    // turnController.setTolerance(Math.toRadians(15));

    driveFeedforward = new SimpleMotorFeedforward(DrivetrainConstants.driveKS, DrivetrainConstants.driveKV);
    driveController = new PIDController(DrivetrainConstants.driveKP, DrivetrainConstants.driveKI, DrivetrainConstants.driveKD);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.idleMode(IdleMode.kBrake);
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** Returns the current state of the module. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getAbsolutePosition().getValueAsDouble()));
  }

  public double toRPM(double velocityMPS){
    return ((velocityMPS * 60)/(2*Math.PI*Constants.DrivetrainConstants.wheelRadius))/6.75;
  }

  /** Returns current position of the module */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(turnEncoder.getAbsolutePosition().getValueAsDouble()));
  }

  public double getTurnPosRadians() {
    return (turnEncoder.getAbsolutePosition().getValueAsDouble())*Math.PI;
  }

  /** Sets desired state of the module (calculated in **RADIANS!!!**) */
  public void setDesiredState(SwerveModuleState desiredState, boolean printStatus) {

    desiredState.optimize(new Rotation2d(getTurnPosRadians()));
    double driveVoltage = driveController.calculate(driveEncoder.getVelocity(), toRPM(desiredState.speedMetersPerSecond)) + driveFeedforward.calculate(desiredState.speedMetersPerSecond);
    double turnVoltage = turnController.calculate(getTurnPosRadians(), desiredState.angle.getRadians()/2);

    driveVoltage = MathUtil.clamp(driveVoltage, -12, 12);
    turnVoltage = MathUtil.clamp(turnVoltage, -12, 12);

    if (printStatus) {
      // SmartDashboard.putNumber("Desired Rot:", desiredState.angle.getRadians()/2);
      // SmartDashboard.putNumber("Current Rot:", getTurnPosRadians());
      // SmartDashboard.putNumber("Error:", turnController.getError());
      SmartDashboard.putNumber("Desired Speed:", toRPM(desiredState.speedMetersPerSecond));
      SmartDashboard.putNumber("Current Speed:", driveEncoder.getVelocity());
      SmartDashboard.putNumber("Drive Voltage:", driveVoltage);
      //SmartDashboard.putNumber("Turn Voltage:", turnVoltage);
    }

    driveMotor.set(desiredState.speedMetersPerSecond/DrivetrainConstants.maxVelocity);
    // driveMotor.setVoltage(driveVoltage);
    turnMotor.setVoltage(-turnVoltage);
  }
}
// PID Tuning (More General):
// https://www.youtube.com/watch?v=u0HFX_iuK5g
// I really liked this video as it capture the behaviors of for PID constants much better:
// https://www.youtube.com/watch?v=6EcxGh1fyMw&t=681s&pp=ygUPSG93IHRvIHR1bmUgcGlk
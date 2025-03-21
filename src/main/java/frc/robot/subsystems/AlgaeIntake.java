// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntake extends SubsystemBase {
  private final SparkMax baseMotor;
  private final SparkMax wheelMotor;

  private final SparkMaxConfig baseConfig;

  private final AbsoluteEncoder baseEncoder;
  private final SparkClosedLoopController PIDController;

  public AlgaeIntake() {
    this.baseMotor = new SparkMax(DeviceIds.algaeBase, MotorType.kBrushless);
    this.wheelMotor = new SparkMax(DeviceIds.algaeWheel, MotorType.kBrushless);

    this.baseConfig = new SparkMaxConfig();
    this.baseConfig.idleMode(IdleMode.kBrake);
    this.baseConfig.inverted(false);
    this.baseConfig.encoder.positionConversionFactor(1/4096);
    this.baseConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    this.baseConfig.closedLoop.pid(1.0,0,0);

    this.baseMotor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.baseEncoder = baseMotor.getAbsoluteEncoder();
    this.PIDController = baseMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("AlgaeIntakePosition", baseEncoder.getPosition());
  }

  public void extend() {
    PIDController.setReference(0.25, ControlType.kPosition);
    this.intake();
  }

  public void retract() {
    PIDController.setReference(0, ControlType.kPosition);
    wheelMotor.setVoltage(0);
  }

  public void intake() {
    wheelMotor.setVoltage(AlgaeIntakeConstants.intakeVoltage);
    // todo: add limit switch detection here
  }

  public void score(){
    wheelMotor.setVoltage(AlgaeIntakeConstants.scoreVoltage);
  }
}

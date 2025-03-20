// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TiltConstants; 
import com.revrobotics.spark.config.SparkMaxConfig;

public class Tilt extends SubsystemBase {
  final double TARGET_DEADBAND = 2;

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController PIDController;

  private final SimpleMotorFeedforward feedforward;

  public double setpoint;

  public Tilt() {
    this.motor = new SparkMax(Constants.DeviceIds.tilt, MotorType.kBrushless);
    this.encoder = motor.getEncoder();
    this.PIDController =  new PIDController(TiltConstants.tiltP, TiltConstants.tiltI, TiltConstants.tiltD);
    this.feedforward = new SimpleMotorFeedforward(TiltConstants.kS, 0);

    final SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.idleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Tilt At Setpoint", this.atTarget());
  }

  public double getPosDegrees() {
    return -(this.encoder.getPosition() * TiltConstants.posConversionFactor);
  }
 
  public void setVoltage(double voltage) {
    this.motor.setVoltage(voltage);
  }
 
  public void setPosition(double desiredPos) {
    // double voltage = desiredPos<0 ? (this.PIDController.calculate(this.getPosDegrees(), desiredPos) + TiltConstants.kS) : (this.PIDController.calculate(this.getPosDegrees(), desiredPos) - TiltConstants.kS);
    final double voltage = MathUtil.clamp((this.PIDController.calculate(this.getPosDegrees(), desiredPos) + TiltConstants.kS) + this.feedforward.calculate(desiredPos), -2, 2);
    this.setpoint = desiredPos;
    this.motor.setVoltage(-voltage);
    SmartDashboard.putNumber("Tilt Error", this.PIDController.getError());
    SmartDashboard.putNumber("Tilt Position", this.getPosDegrees());
    SmartDashboard.putNumber("Tilt Setpoint", this.setpoint);
    SmartDashboard.putNumber("Tilt Voltage", -voltage);
  }

  public boolean atTarget() {
    if (this.getPosDegrees() < this.setpoint + TARGET_DEADBAND && this.getPosDegrees() > this.setpoint - TARGET_DEADBAND) {
      return true;
    }
    return false;
  }

  public void stop() {
    this.motor.setVoltage(0);
  }

  public Command setPositionCommand(double desired) {
    return this.run(() -> this.setPosition(desired));
  }
}
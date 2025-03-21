// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final PIDController pidController;
  private final RelativeEncoder encoder;
  private final SimpleMotorFeedforward feedforward;
  public double setpoint;

  /** i stole this from the internet and it seems smart */
  // public enum ElevatorPosition {
  //   LEVEL_1(ElevatorConstants.l1Score),
  //   LEVEL_2(ElevatorConstants.l2Score),
  //   LEVEL_3(ElevatorConstants.l3Score),
  //   LEVEL_4(ElevatorConstants.l4Score);

  //   public final double positionInches;

  //   ElevatorPosition(double positionInches) {
  //     this.positionInches = positionInches;
  //   }
  // }

  public Elevator() {
    this.rightMotor = new SparkMax(DeviceIds.rightElevator, MotorType.kBrushless);
    this.leftMotor = new SparkMax(DeviceIds.leftElevator, MotorType.kBrushless);

    this.pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    
    this.encoder = leftMotor.getEncoder();
    this.feedforward = new SimpleMotorFeedforward(ElevatorConstants.kS, 0);
    this.setpoint = 0;
  }

  @Override
  public void periodic() {
    this.setPosition(this.setpoint);
    SmartDashboard.putBoolean("Elevator At Setpoint", this.atTarget());
  }

  public double getPosInches() {
    return this.encoder.getPosition()*ElevatorConstants.positionConversionFactor;
  }

  public void setPosition(double position) {
    final double voltage = MathUtil.clamp(this.pidController.calculate(-this.encoder.getPosition(), position) + feedforward.calculate(-this.encoder.getPosition()), -2, 2);

    SmartDashboard.putNumber("Elevator Error", this.pidController.getError());
    SmartDashboard.putNumber("Elevator Position", -this.encoder.getPosition());
    SmartDashboard.putNumber("Elevator Setpoint", position);
    SmartDashboard.putNumber("Elevator Voltage", voltage);

    this.setpoint = position;

    this.setVoltage(voltage);
  }

  public void setVoltage(double voltage) {
    this.leftMotor.setVoltage(-voltage);
    this.rightMotor.setVoltage(-voltage);
  }

  public void stop() {
    this.leftMotor.setVoltage(0);
    this.rightMotor.setVoltage(0);
  }

  /** Returns whether the elevator has reached the setpoint */
  public boolean atTarget() {
    if (-this.encoder.getPosition() < this.setpoint + ElevatorConstants.targetDeadband && -this.encoder.getPosition() > this.setpoint - ElevatorConstants.targetDeadband) {
      return true;
    }
    return false;
  }

  // public Command setPositionCommand(double pos) {
  //   return this.run(() -> this.setPosition(pos));
  // }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final PIDController pidController;
  private final AbsoluteEncoder encoder;
  private final double offset;
  private double setpoint;

  /** i stole this from the internet and it seems smart */
  public enum ElevatorPosition {
    LEVEL_1(ElevatorConstants.l1Score),
    LEVEL_2(ElevatorConstants.l2Score),
    LEVEL_3(ElevatorConstants.l3Score),
    LEVEL_4(ElevatorConstants.l4Score);

    public final double positionInches;

    ElevatorPosition(double positionInches) {
      this.positionInches = positionInches;
    }
  }

  public Elevator() {
    rightMotor = new SparkMax(DeviceIds.rightElevator, MotorType.kBrushless);
    leftMotor = new SparkMax(DeviceIds.leftElevator, MotorType.kBrushless);

    pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    
    encoder = leftMotor.getAbsoluteEncoder();
    offset = encoder.getPosition();
  }

  @Override
  public void periodic() {
    setPosition(this.setpoint);
    SmartDashboard.putNumber("Elevator Position", this.getPosInches());
    SmartDashboard.putNumber("Elevator Setpoint", this.setpoint);
  }

  public double getPosInches() {
    return (this.encoder.getPosition()-this.offset)*ElevatorConstants.positionConversionFactor;
  }

  public void setPosition(double position) {
    final double voltage = this.pidController.calculate(this.getPosInches(), position);
    this.setpoint = position; //? if we are setting the position, we don't necessarily always want to also change the setpoint, right?
    this.setVoltage(voltage);
  }

  public void setVoltage(double voltage) {
    this.leftMotor.setVoltage(voltage);
    this.rightMotor.setVoltage(-voltage);
  }

  public void stop() {
    this.leftMotor.setVoltage(0);
    this.rightMotor.setVoltage(0);
  }

  /** Returns whether the elevator has reached the setpoint */
  public boolean atTarget() {
    return (this.getPosInches()==this.setpoint);
  }

  public Command setPositionCommand(double pos) {
    return this.run(() -> this.setPosition(pos));
  }
}

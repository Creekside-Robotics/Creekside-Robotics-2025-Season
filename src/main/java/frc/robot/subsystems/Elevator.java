// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  
  SparkMax leftMotor;
  SparkMax rightMotor;
  PIDController pidController;
  AbsoluteEncoder encoder;
  double offset;
  double setpoint;

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

  public double getPosInches(){
    return (this.encoder.getPosition()-this.offset)*ElevatorConstants.positionConversionFactor;
  }

  public void setPosition(double position) {
    double voltage = this.pidController.calculate(this.getPosInches(), position);
    this.setpoint = position;
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

  public boolean atTarget() {
    return (this.getPosInches()==this.setpoint);
  }

  public Command setPositionCommand(double pos){
    return this.run(() -> this.setPosition(pos));
  }
}

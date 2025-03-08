// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TiltConstants; 
import com.revrobotics.spark.config.SparkMaxConfig;



public class Tilt extends SubsystemBase {
  private final SparkMax motor;
  private final SparkAbsoluteEncoder encoder;
  private final PIDController PIDController;
  private double desiredPos;

  public Tilt(){
    this.motor = new SparkMax(Constants.DeviceIds.tilt,  MotorType.kBrushless);
    this.encoder = motor.getAbsoluteEncoder();
    this.PIDController =  new PIDController(TiltConstants.tiltP,TiltConstants.tiltI, TiltConstants.tiltD);

    final SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.idleMode(IdleMode.kBrake);

    PIDController.enableContinuousInput(-Math.PI, Math.PI);
  }


  public double getPosRadians() {
    return (this.encoder.getPosition() - TiltConstants.positionOffset)*Math.PI;
  }
 
  public void setVoltage(double voltage){
    this.motor.setVoltage(voltage);
  }
 
  public void setPosition(double desiredPos){
    double voltage = this.PIDController.calculate(this.getPosRadians(), desiredPos);
    this.desiredPos = desiredPos;
    this.motor.setVoltage(voltage);
  }

  public boolean atPosition(){
    return (this.getPosRadians() == this.desiredPos);
  }

  public void stop(){
    this.motor.setVoltage(0);
  }

  public Command setPosCommand(double desired){
    return this.run(() -> this.setPosition(desired));
  }
}
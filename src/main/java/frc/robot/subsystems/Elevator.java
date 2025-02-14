// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  
  SparkMax leftMotor;
  SparkMax rightMotor;
  SparkMaxConfig leftConfig;
  SparkMaxConfig rightConfig;
  SparkClosedLoopController rightPIDController;
  SparkClosedLoopController leftPIDController;
  AbsoluteEncoder encoder;

  public Elevator(){
    rightMotor = new SparkMax(Constants.DeviceIds.rightElevator, MotorType.kBrushless);
    leftMotor = new SparkMax(Constants.DeviceIds.leftElevator, MotorType.kBrushless);

    rightConfig = new SparkMaxConfig();
    leftConfig = new SparkMaxConfig();

    rightPIDController = rightMotor.getClosedLoopController();
    leftPIDController = leftMotor.getClosedLoopController();

  }

  // @Override
  // public void periodic() {
  //   setVoltage(
  //     MathUtil.clamp(
  //       encoder.getPosition(), 
  //       -TiltConstants.maxVoltage, 
  //       TiltConstants.maxVoltage
  //     ) + TiltConstants.kS * Math.sin(encoder.getPosition() - TiltConstants.hangingAngle)
  //   );
  //   SmartDashboard.putNumber("Tilt Position", this.encoder.getPosition());
  // }

  public void setPosition(double position){
    leftPIDController.setReference(position, ControlType.kPosition);
    rightPIDController.setReference(position, ControlType.kPosition);
  }

  public void setVoltage(double voltage){
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(-voltage);
  }

  public void stop(){
    leftMotor.setVoltage(0);
    rightMotor.setVoltage(0);
  }

  public boolean atTarget(){
    return false;
  }
}

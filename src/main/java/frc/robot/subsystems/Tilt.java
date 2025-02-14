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

public class Tilt extends SubsystemBase {
  
  SparkMax mainMotor;
  SparkMaxConfig mainConfig;
  SparkClosedLoopController PIDController;
  AbsoluteEncoder encoder;

  public Tilt(){
    this.mainMotor = new SparkMax(Constants.DeviceIds.tilt, MotorType.kBrushless);
    this.mainConfig = new SparkMaxConfig();

    this.PIDController = mainMotor.getClosedLoopController();
    //Set up through bore encoder

    this.mainConfig.idleMode(IdleMode.kBrake);
    //mainConfig.closedLoop.pid(0, 0, 0);
    this.mainMotor.configure(mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //@Override
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
    this.PIDController.setReference(position, ControlType.kPosition);
  }

  public void setVoltage(double voltage){
    this.mainMotor.setVoltage(voltage);
  }

  public boolean atTarget(){
    return true;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  
  SparkMax leftGrabber;
  SparkMax rightGrabber;
  SparkMaxConfig config;


  public Arm() {
    rightGrabber = new SparkMax(DeviceIds.rightGrabber, MotorType.kBrushless);
    leftGrabber = new SparkMax(DeviceIds.leftGrabber, MotorType.kBrushless);
    
    config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    
    rightGrabber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.inverted(true);
    leftGrabber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intake(){
    rightGrabber.setVoltage(ArmConstants.intakeVoltage);
    leftGrabber.setVoltage(-ArmConstants.intakeVoltage);
  }

  public void outtake(){
    rightGrabber.setVoltage(-ArmConstants.intakeVoltage);
    leftGrabber.setVoltage(ArmConstants.intakeVoltage);
  }

  public void stop(){
    rightGrabber.setVoltage(0);
    leftGrabber.setVoltage(0);
  }
}

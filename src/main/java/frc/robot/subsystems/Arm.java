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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  
  SparkMax leftGrabber;
  SparkMax rightGrabber;
  SparkMaxConfig config;
  DigitalInput limitSwitch;

  public Arm() {
    this.rightGrabber = new SparkMax(DeviceIds.rightGrabber, MotorType.kBrushless);
    this.leftGrabber = new SparkMax(DeviceIds.leftGrabber, MotorType.kBrushless);

    this.limitSwitch = new DigitalInput(DeviceIds.armLimitSwitch);
    
    this.config = new SparkMaxConfig();
    this.config.inverted(false);
    this.config.idleMode(IdleMode.kBrake);
    
    this.rightGrabber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.config.inverted(true);
    this.leftGrabber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("Has Coral", hasCoral());
  }

  public boolean hasCoral() {
    return this.limitSwitch.get();
  }

  public void intake() {
    rightGrabber.setVoltage(-ArmConstants.intakeVoltage);
    leftGrabber.setVoltage(-ArmConstants.intakeVoltage);
    if (hasCoral()) {
      this.stop();
    }
  }

  public void outtake() {
    rightGrabber.setVoltage(ArmConstants.intakeVoltage);
    leftGrabber.setVoltage(ArmConstants.intakeVoltage);
  }

  public void stop() {
    rightGrabber.setVoltage(0);
    leftGrabber.setVoltage(0);
  }

  public Command intakeCommand() {
    return this.runEnd(() -> this.intake(), () -> this.stop());
  }

  public Command outtakeCommand() {
    return this.runEnd(() -> this.outtake(), () -> this.stop());
  }
}

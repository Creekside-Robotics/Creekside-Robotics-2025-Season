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
  
  SparkMax baseMotor;
  SparkMax wheelMotor;

  SparkMaxConfig baseConfig;

  AbsoluteEncoder baseEncoder;
  SparkClosedLoopController PIDController;


  public AlgaeIntake(){
    baseMotor = new SparkMax(DeviceIds.algaeBase, MotorType.kBrushless);
    wheelMotor = new SparkMax(DeviceIds.algaeWheel, MotorType.kBrushless);

    baseConfig.idleMode(IdleMode.kBrake);
    baseConfig.inverted(false);
    baseConfig.encoder.positionConversionFactor(1/4096);
    baseConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    baseConfig.closedLoop.pid(1.0,0,0);

    baseMotor.configure(baseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    baseEncoder = baseMotor.getAbsoluteEncoder();
    PIDController = baseMotor.getClosedLoopController();
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("AlgaeIntakePosition", baseEncoder.getPosition());
  }

  public void extend(){
    PIDController.setReference(0.25, ControlType.kPosition);
    intake();
  }

  public void retract(){
    PIDController.setReference(0, ControlType.kPosition);
    wheelMotor.setVoltage(0);
  }

  public void intake(){
    wheelMotor.setVoltage(AlgaeIntakeConstants.intakeVoltage);
    //add limit switch detection here
  }

  public void score(){
    wheelMotor.setVoltage(AlgaeIntakeConstants.scoreVoltage);
  }

}

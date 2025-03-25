// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class DriveAuto extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drivetrain swerveDrive;

  /**
   * Constructs a new Drive instance. Handles getting joystick inputs and setting swerve state. 
   * @param swerveDrive the drivetrain
   * @param joystick the jotstick
   */
  public DriveAuto(Drivetrain swerveDrive) {
    this.swerveDrive = swerveDrive;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.swerveDrive.m_frontRight.setDesiredState(new SwerveModuleState(DrivetrainConstants.autoSpeed, new Rotation2d()), false);
    this.swerveDrive.m_frontLeft.setDesiredState(new SwerveModuleState(DrivetrainConstants.autoSpeed, new Rotation2d()), false);
    this.swerveDrive.m_backRight.setDesiredState(new SwerveModuleState(DrivetrainConstants.autoSpeed, new Rotation2d()), false);
    this.swerveDrive.m_backLeft.setDesiredState(new SwerveModuleState(DrivetrainConstants.autoSpeed, new Rotation2d()), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

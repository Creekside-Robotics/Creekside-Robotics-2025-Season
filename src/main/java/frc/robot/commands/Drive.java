// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class Drive extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drivetrain swerveDrive;
  private final CommandJoystick joystick;
  private boolean fieldRelative = true;

  /**
   * Constructs a new Drive instance. Handles getting joystick inputs and setting swerve state. 
   * @param swerveDrive the drivetrain
   * @param joystick the jotstick
   */
  public Drive(Drivetrain swerveDrive, CommandJoystick joystick) {
    this.swerveDrive = swerveDrive;
    this.joystick = joystick;

    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = -MathUtil.applyDeadband(this.joystick.getY(), 0.1);
    SmartDashboard.putNumber("X Input", this.joystick.getY());
    double yVelocity = -MathUtil.applyDeadband(this.joystick.getX(), 0.1);
    SmartDashboard.putNumber("Y Input", yVelocity);
    double angVelocity = -MathUtil.applyDeadband(this.joystick.getZ(), 0.1) * DrivetrainConstants.maxAngularVelocity;
    SmartDashboard.putNumber("Rot Input", this.joystick.getZ());

    if (joystick.button(3).getAsBoolean()) {
      DrivetrainConstants.maxVelocityMultiplier = -0.075;
      fieldRelative = false;
    }else{
      DrivetrainConstants.maxVelocityMultiplier = (this.joystick.getThrottle()+1)/4;
      fieldRelative =  true;
    }
  
    SmartDashboard.putNumber("Velocity Multiplier", DrivetrainConstants.maxVelocityMultiplier);

    this.swerveDrive.setModuleStates(xVelocity, yVelocity, angVelocity, fieldRelative);
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

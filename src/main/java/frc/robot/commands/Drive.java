// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class Drive extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drivetrain m_swerveDrive;
  private final CommandJoystick m_joystick;
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  /**
   * @param subsystem The subsystem used by this command.
   */
  public Drive(Drivetrain swerveDrive, CommandJoystick joystick) {
    m_swerveDrive = swerveDrive;
    m_joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xVelocity = -MathUtil.applyDeadband(m_joystick.getY(), 0.1) * DrivetrainConstants.maxVelocity;
    SmartDashboard.putNumber("X Input", m_joystick.getY());
    // System.out.println("Forward Value: " + xVelocity);
    double yVelocity = -MathUtil.applyDeadband(m_joystick.getX(), 0.1) * DrivetrainConstants.maxVelocity;
    SmartDashboard.putNumber("Y Input", m_joystick.getX());
    // System.out.println("Stafe Value: " + yVelocity);
    double angVelocity = -MathUtil.applyDeadband(m_joystick.getZ(), 0.1) * DrivetrainConstants.maxAngularVelocity;
    SmartDashboard.putNumber("Rot Input", m_joystick.getZ());
    // System.out.println("Rot Value: " + angVelocity);


    m_swerveDrive.setModuleStates(xVelocity, yVelocity, angVelocity, true);
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

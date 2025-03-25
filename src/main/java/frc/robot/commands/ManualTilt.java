// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Tilt;

/** This isn't in the Tilt class because we need isFinished and it doesn't look like we can do that there */
public class ManualTilt extends Command {
  /** shut up */
  public static boolean enabled = false;

  private final Tilt tilt;
  private final CommandJoystick joystick;

  public ManualTilt(Tilt tilt, CommandJoystick joystick) {
    this.tilt = tilt;
    this.joystick = joystick;

    addRequirements(tilt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ManualTilt.enabled) {
      this.tilt.setPosition(this.joystick.getX()*60+75);
      SmartDashboard.putNumber("Tilt Input", this.joystick.getX()*60+75);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // we are always running this, only buttons change whether it does anything
  }
}

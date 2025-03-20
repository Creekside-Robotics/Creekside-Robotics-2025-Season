// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/** This isn't in the Elevator class because we need isFinished and it doesn't look like we can do that there */
public class SetElevatorPosition extends Command {

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Elevator elevator;
  private final double position;

  public SetElevatorPosition(Elevator elevator, double position) {
    this.elevator = elevator;
    this.position = position;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.elevator.setPosition(this.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.elevator.atTarget();
  }
}

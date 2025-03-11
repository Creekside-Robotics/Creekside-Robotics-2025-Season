// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TiltConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tilt;

/** An example command that uses an example subsystem. */
public class PickupCoral extends Command {
  private final Elevator elevator;
  private final Arm arm;
  private final Tilt tilt;

  public PickupCoral(Elevator elevator, Arm arm, Tilt tilt) {
    this.elevator = elevator;
    this.arm = arm;
    this.tilt = tilt;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.elevator.setPosition(ElevatorConstants.intakePostitionStart);
    this.arm.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.tilt.setPosition(TiltConstants.intakePostition);
    this.arm.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.arm.hasCoral();
  }
}

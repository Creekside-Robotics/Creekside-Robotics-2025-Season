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

  private final Elevator Elevator;
  private final Arm Arm;
  private final Tilt Tilt;

  public PickupCoral(Elevator elevator, Arm arm, Tilt tilt){
    this.Elevator = elevator;
    this.Arm = arm;
    this.Tilt = tilt;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.Tilt.setPosition(TiltConstants.intakePostition);
    this.Arm.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.Arm.intake();
    this.Elevator.setPosition(ElevatorConstants.pickupPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.Arm.hasCoral();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveAuto;
import frc.robot.commands.ManualTilt;
import frc.robot.commands.PickupCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.SetDefault;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tilt;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final Drivetrain drivetrain = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  public final Tilt tilt = new Tilt();

  private final CommandJoystick driverController = new CommandJoystick(Constants.DeviceIds.driver1Port);
  private final CommandJoystick backupController = new CommandJoystick(Constants.DeviceIds.driver2Port);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    this.drivetrain.setDefaultCommand(new Drive(drivetrain, driverController));

    //reset gyro
    this.driverController.button(7).onTrue(drivetrain.commandResetGyro());
    this.backupController.button(7).onTrue(drivetrain.commandResetGyro());

    //reset tower
    this.driverController.button(4).whileTrue(new SetDefault(elevator, tilt));
    this.backupController.button(4).whileTrue(new SetDefault(elevator, tilt));

    //intake coral
    this.driverController.button(2).whileTrue(new PickupCoral(elevator, arm, tilt));
    this.driverController.button(2).onFalse(new SetDefault(elevator, tilt));
  
    //Score coral (Trigger, 11:L1&2, 12:L3)
    this.driverController.button(1).whileTrue(new ScoreCoral(elevator, arm, tilt, ElevatorConstants.l4Score));
    this.driverController.button(1).onFalse(new SetDefault(elevator, tilt));

    this.driverController.button(11).whileTrue(new ScoreCoral(elevator, arm, tilt, ElevatorConstants.l1Score));
    this.driverController.button(11).onFalse(new SetDefault(elevator, tilt));
    
    this.driverController.button(12).whileTrue(new ScoreCoral(elevator, arm, tilt, ElevatorConstants.l3Score));
    this.driverController.button(12).onFalse(new SetDefault(elevator, tilt));

    //Intake and outtake coral
    this.driverController.button(9).whileTrue(arm.intakeCommand());
    this.driverController.button(10).whileTrue(arm.outtakeCommand());
    this.backupController.button(9).whileTrue(arm.intakeCommand());
    this.backupController.button(10).whileTrue(arm.outtakeCommand());


    //Direct control arm NO EXPLAINATION NECESSARY
    this.backupController.button(3).whileTrue(new ManualTilt(tilt, backupController));
    this.backupController.button(3).onFalse(new SetDefault(elevator, tilt));
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new DriveAuto(drivetrain);
  }
}

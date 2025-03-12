package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TiltConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tilt;

public class PickupCoral extends SequentialCommandGroup {
    PickupCoral(Elevator elevator, Arm arm, Tilt tilt) {
        arm.stop();
        addCommands(
            elevator.setPositionCommand(ElevatorConstants.intakePostitionStart),
            tilt.setPositionCommand(TiltConstants.intakePostition),
            arm.intakeCommand()
        );
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tilt;

public class SetDefault extends SequentialCommandGroup {
    public SetDefault(Elevator elevator, Tilt tilt) {
        addCommands(
            new SetTiltPosition(tilt, 15),
            new SetElevatorPosition(elevator, 0)
        );
    }
}

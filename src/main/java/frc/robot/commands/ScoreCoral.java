package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TiltConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Tilt;

public class ScoreCoral extends SequentialCommandGroup {
    public ScoreCoral(Elevator elevator, Arm arm, Tilt tilt, ElevatorPosition position) {
        arm.stop();
        addCommands(
            elevator.setPositionCommand(position.positionInches),
            tilt.setPositionCommand(TiltConstants.scorePosition)
        );
    }
}

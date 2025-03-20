package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TiltConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tilt;

public class ScoreCoral extends SequentialCommandGroup {
    public ScoreCoral(Elevator elevator, Arm arm, Tilt tilt, double position) {
        arm.stop();
        addCommands(
            new SetElevatorPosition(elevator, position),
            new SetTiltPosition(tilt, TiltConstants.scorePosition)
        );
    }
}

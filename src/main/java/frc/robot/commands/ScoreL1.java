package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.TiltConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tilt;

public class ScoreL1 extends SequentialCommandGroup {
    public ScoreL1(Elevator elevator, Arm arm, Tilt tilt) {
        arm.stop();
        addCommands(
            new SetElevatorPosition(elevator, ElevatorConstants.l1Score),
            new SetTiltPosition(tilt, TiltConstants.scoreL1Position)
        );
    }
}

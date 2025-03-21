

public class ScoreL1 extends SequentialCommandGroup {
    public ScoreCoral(Elevator elevator, Arm arm, Tilt tilt, double position) {
        arm.stop();
        addCommands(
            new SetElevatorPosition(elevator, position),
            new SetTiltPosition(tilt, TiltConstants.scorePosition)
        );
    }
}

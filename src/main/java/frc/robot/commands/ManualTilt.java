package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Tilt;

public class ManualTilt extends SetTiltPosition {
    public ManualTilt(Tilt tilt, CommandJoystick joystick){
        super(tilt, joystick.getY()*60+75);
    }
}

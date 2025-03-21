package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Tilt;

public class ManualTilt extends RepeatCommand {
    public ManualTilt(Tilt tilt, CommandJoystick joystick){
        super(new SetTiltPosition(tilt, joystick.getX()*60+75));
        SmartDashboard.putNumber("Manual Tilt Input", joystick.getX()*60+75);
        SmartDashboard.putNumber("Y-Input", joystick.getX());
    }
}

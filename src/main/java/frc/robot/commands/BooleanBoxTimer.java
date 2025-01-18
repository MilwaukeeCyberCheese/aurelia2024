package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.WaitCommandMilli;

public class BooleanBoxTimer extends SequentialCommandGroup {
    public BooleanBoxTimer() {
        addCommands(new WaitCommandMilli(500), new Command() {
            @Override
            public void initialize() {
                SmartDashboard.putBoolean("Shoot!", true);
            }
        });
    }
}

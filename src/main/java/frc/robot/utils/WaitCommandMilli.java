package frc.robot.utils;

public class WaitCommandMilli extends edu.wpi.first.wpilibj2.command.WaitCommand{

    //replace generic wait command with command in millisecondsw
    public WaitCommandMilli(double milliseconds) {
        super(milliseconds / 1000);
    }
    
}

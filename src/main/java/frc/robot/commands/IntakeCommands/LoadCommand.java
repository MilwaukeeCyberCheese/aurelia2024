package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class LoadCommand extends Command {
    private IntakeSubsystem m_intakeSubsystem;

    public LoadCommand(IntakeSubsystem intakeSubsystem){
        m_intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute(){
        m_intakeSubsystem.setSpeed(Constants.IntakeConstants.kLoadSpeed);
    }

    @Override
    public boolean isFinished(){
        int[][] colors = {{Constants.Sensors.intakeColorSensor.getRed(), Constants.Sensors.intakeColorSensor.getGreen(), Constants.Sensors.intakeColorSensor.getBlue()}, Constants.IntakeConstants.kNoteColors};
        
        for(int[] color : colors){
            if(Math.abs(color[0] - color[1]) > Constants.IntakeConstants.kColorTolerance){
                return true;
            }
        }

        return false;
    }
}
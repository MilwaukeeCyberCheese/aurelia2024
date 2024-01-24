package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setSpeed(Constants.IntakeConstants.kIntakeSpeed);
    }

    @Override
    public boolean isFinished(){
        int[][] colors = {{Constants.Sensors.intakeColorSensor.getRed(), Constants.Sensors.intakeColorSensor.getGreen(), Constants.Sensors.intakeColorSensor.getBlue()}, Constants.IntakeConstants.kNoteColors};
        
        for(int[] color : colors){
            if(Math.abs(color[0] - color[1]) > Constants.IntakeConstants.kColorTolerance){
                return false;
            }
        }

        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}
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
        return true; //TODO
    }
}
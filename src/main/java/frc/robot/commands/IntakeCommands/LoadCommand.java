package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class LoadCommand extends Command {
    private IntakeSubsystem m_intakeSubsystem;

    /**
     * Command to run the intake at the speed for loading into the shooter
     * 
     * @param intakeSubsystem
     */
    public LoadCommand(IntakeSubsystem intakeSubsystem){
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    @Override
    public void execute(){
        m_intakeSubsystem.setSpeed(Constants.IntakeConstants.kLoadSpeed);
    }

    @Override
    public boolean isFinished(){
        return false; //TODO
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}
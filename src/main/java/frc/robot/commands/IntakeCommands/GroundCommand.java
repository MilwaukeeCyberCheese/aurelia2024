package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class GroundCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;

    /**
     * Command to make the intake go on the ground
     * 
     * @param intakeSubsystem
     */
    public GroundCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setPivotPosition(Constants.IntakeConstants.kPivotGroundPosition);
    }

    @Override
    public boolean isFinished(){
        return m_intakeSubsystem.atPosition();
    }
}
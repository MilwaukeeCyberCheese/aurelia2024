package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class StowCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;

    public StowCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setPivotPosition(Constants.IntakeConstants.kPivotStowPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
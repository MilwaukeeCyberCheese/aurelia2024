package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class PivotCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final DoubleSupplier m_position;

    public PivotCommand(DoubleSupplier position, IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        m_position = position;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setPivotPosition(m_position.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
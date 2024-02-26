package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePositionCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final DoubleSupplier m_position;

    /**
     * Command to set the position of the intake
     * 
     * @param position angle to set the intake to (0 is ground position)
     * @param intakeSubsystem
     */
    public IntakePositionCommand(DoubleSupplier position, IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        m_position = position;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setPosition(m_position.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return m_intakeSubsystem.atPosition();
    }
}
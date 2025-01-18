package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class SetIntakeSpeed extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final DoubleSupplier m_speed;

    /**
     * Command to set the speed of the intake
     * 
     * @param speed
     * @param intakeSubsystem
     */
    public SetIntakeSpeed(DoubleSupplier speed, IntakeSubsystem intakeSubsystem) {
        m_speed = speed;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setSpeed(m_speed.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}
package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SetSpinBoth extends Command {
    public final DoubleSupplier m_upperRPM;
    public final DoubleSupplier m_lowerRPM;
    public final ShooterSubsystem m_shooterSubsystem;

    /**
     * Runs the shooter at a specific RPM
     * 
     * @param RPM              RPM to spin the shooter at
     * @param ShooterSubsystem
     */
    public SetSpinBoth(DoubleSupplier upperRPM, DoubleSupplier lowerRPM, ShooterSubsystem shooterSubsystem) {
        m_upperRPM = upperRPM;
        m_lowerRPM = lowerRPM;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void execute() {
        m_shooterSubsystem.setRPM(m_upperRPM.getAsDouble(), m_lowerRPM.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return m_shooterSubsystem.atRPM();
    }
}

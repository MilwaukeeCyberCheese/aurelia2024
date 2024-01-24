package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpCommand extends Command {
    public final DoubleSupplier m_rpm;
    public final ShooterSubsystem m_shooterSubsystem;

    /**
     * Runs the shooter at a specific RPMs
     * 
     * @param RPM              RPMs to spin the shooter up to
     * @param ShooterSubsystem subsystem for controlling the shooter
     *                         {@link frc.robot.subsystems.ShooterSubsystem link}
     */
    public SpinUpCommand(DoubleSupplier RPM, ShooterSubsystem shooterSubsystem) {
        m_rpm = RPM;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    public void execute() {
        m_shooterSubsystem.setRPM(m_rpm.getAsDouble());
    }

    public boolean isFinished() {
        return m_shooterSubsystem.atRPM();
    }
}

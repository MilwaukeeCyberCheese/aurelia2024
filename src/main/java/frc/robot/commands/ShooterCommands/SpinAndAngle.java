package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinAndAngle extends Command {
    public final ShooterSubsystem m_shooterSubsystem;
    public final DoubleSupplier m_angle;
    public final DoubleSupplier m_rpm;

    /**
     * Set angle of the shooter
     * 
     * @param angle            angle to set the shooter to
     * @param shooterSubsystem subsystem for controlling the shooter
     *                         {@link frc.robot.subsystems.ShooterSubsystem link}
     */
    public SpinAndAngle(DoubleSupplier angle, DoubleSupplier RPM, ShooterSubsystem shooterSubsystem) {
        m_angle = angle;
        m_rpm = RPM;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void execute() {
        m_shooterSubsystem.setPosition(m_angle.getAsDouble());
        m_shooterSubsystem.setRPM(m_rpm.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return m_shooterSubsystem.atPosition() && m_shooterSubsystem.atRPM();
    }
}

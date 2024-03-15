package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SetSpinAndAngle extends Command {
    public final ShooterSubsystem m_shooterSubsystem;
    public final DoubleSupplier m_angle;
    public final DoubleSupplier m_upperRPM;
    public final DoubleSupplier m_lowerRPM;

    /**
     * Set angle of the shooter
     * 
     * @param angle            angle to set the shooter to
     * @param upperRPM         RPM to set the upper shooter to
     * @param lowerRPM         RPM to set the lower shooter to
     * @param shooterSubsystem
     */
    public SetSpinAndAngle(DoubleSupplier angle, DoubleSupplier upperRPM, DoubleSupplier lowerRPM,
            ShooterSubsystem shooterSubsystem) {
        m_angle = angle;
        m_upperRPM = upperRPM;
        m_lowerRPM = lowerRPM;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void execute() {
        m_shooterSubsystem.setPosition(m_angle.getAsDouble());
        m_shooterSubsystem.setRPM(m_upperRPM.getAsDouble(), m_lowerRPM.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return m_shooterSubsystem.atPosition() && m_shooterSubsystem.atRPM();
    }
}

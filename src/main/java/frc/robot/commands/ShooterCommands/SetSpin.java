package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SetSpin extends Command {
    public final DoubleSupplier m_rpm;
    public final ShooterSubsystem m_shooterSubsystem;

    /**
     * Runs the shooter at a specific RPM
     * 
     * @param RPM              RPM to spin the shooter at
     * @param ShooterSubsystem
     */
    public SetSpin(DoubleSupplier RPM, ShooterSubsystem shooterSubsystem) {
        m_rpm = RPM;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("SetSpeedExecute", m_rpm.getAsDouble());
        m_shooterSubsystem.setRPM(m_rpm.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return m_shooterSubsystem.atRPM();
    }
}

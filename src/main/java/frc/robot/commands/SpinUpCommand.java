package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpCommand extends Command{
    public final DoubleSupplier m_speed;
    public final ShooterSubsystem m_shooterSubsystem;

    /**
     * Runs the shooter at a specific speed
     * 
     * @param speed           speed to run the flywheels at (in RPMs)
     * @param ShooterSubsystem subsystem for controlling the shooter
     *                       {@link frc.robot.subsystems.ShooterSubsystem link}
     */
    public SpinUpCommand(DoubleSupplier speed, ShooterSubsystem shooterSubsystem) {
        m_speed = speed;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    public void execute(){
        m_shooterSubsystem.setSpeed(m_speed.getAsDouble());
    }

    public boolean isFinished(){
        return m_shooterSubsystem.atSpeed();
    }
}

package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShooterAngleCommand extends Command {
    public final ShooterSubsystem m_shooterSubsystem;
    public final DoubleSupplier m_angle;

    /**
     * Set angle of the shooter
     * 
     * @param angle angle to set the shooter to
     * @param ShooterSubsystem subsystem for controlling the shooter
     *                       {@link frc.robot.subsystems.ShooterSubsystem link}
     */
    public SetShooterAngleCommand(DoubleSupplier angle, ShooterSubsystem shooterSubsystem) {
        m_angle = angle;
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    public void execute(){
        m_shooterSubsystem.setPosition(m_angle.getAsDouble());
    }

    public boolean isFinished(){
        return m_shooterSubsystem.atPosition();
    }
}
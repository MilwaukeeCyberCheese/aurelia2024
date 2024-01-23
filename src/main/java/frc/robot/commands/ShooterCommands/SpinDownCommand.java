package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinDownCommand extends Command{
    public final ShooterSubsystem m_shooterSubsystem;

    /**
     * Spins down the shooter to a speed of 0
     * 
     * @param ShooterSubsystem subsystem for controlling the shooter
     *                       {@link frc.robot.subsystems.ShooterSubsystem link}
     */
    public SpinDownCommand(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    public void execute(){
        m_shooterSubsystem.setRPM(0);
    }

    public boolean isFinished(){
        return m_shooterSubsystem.atRPM();
    }
}
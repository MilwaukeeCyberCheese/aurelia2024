package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class LoadShooterCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;

    /**
     * //TODO: copilot!
     * @param shooterSubsystem
     */
    public LoadShooterCommand(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void execute() {
        m_shooterSubsystem.setRPM(Constants.ShooterConstants.kLoadRPM);
    }

    @Override
    public boolean isFinished() {
        return Constants.Sensors.shooterColorSensor.getRed() > Constants.ShooterConstants.kRedNoteDetectionThreshold;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.setRPM(0);
    }
}

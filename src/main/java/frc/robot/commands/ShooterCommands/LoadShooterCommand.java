package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class LoadShooterCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;

    /**
     * Run the shooter slowly to load the note for the amp
     * 
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
        // return Constants.Sensors.shooterColorSensor.getRed() > Constants.ShooterConstants.kRedNoteDetectionThreshold;
        return false;
    }
}

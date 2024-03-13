package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;

    /**
     * Command to intake the note, ends when the limit switch is pressed
     * 
     * @param intakeSubsystem
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setSpeed(Constants.IntakeConstants.kIntakeSpeed);
    }

    @Override
    public boolean isFinished() {
        return Constants.Sensors.intakeLimitSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}
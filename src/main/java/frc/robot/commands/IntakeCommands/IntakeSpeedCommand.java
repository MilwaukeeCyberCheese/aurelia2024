package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSpeedCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final DoubleSupplier m_speed;

    /**
     * Command to intake the note
     * 
     * @param intakeSubsystem
     */
    public IntakeSpeedCommand(DoubleSupplier speed, IntakeSubsystem intakeSubsystem) {
        m_speed = speed;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.setSpeed(m_speed.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return false; //TODO: return limit switch
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}
package frc.robot.commands.LiftCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class SetLiftPosition extends Command {
    public final LiftSubsystem m_liftSubsystem;
    public final DoubleSupplier m_position;

    /**
     * Command to set the lift to a given position
     * 
     * @param position position to set the lift to (measured in inches of travel(not really))
     * @param liftSubsystem
     */
    public SetLiftPosition(DoubleSupplier position, LiftSubsystem liftSubsystem) {
        this.m_position = position;
        this.m_liftSubsystem = liftSubsystem;
        addRequirements(m_liftSubsystem);
    }

    @Override
    public void execute(){
        m_liftSubsystem.setPosition(m_position.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return m_liftSubsystem.atPosition();
    }
}

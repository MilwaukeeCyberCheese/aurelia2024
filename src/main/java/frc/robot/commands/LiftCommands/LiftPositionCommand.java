package frc.robot.commands.LiftCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class LiftPositionCommand extends Command {
    public final LiftSubsystem m_liftSubsystem;
    public final DoubleSupplier m_position;

    public LiftPositionCommand(DoubleSupplier position, LiftSubsystem liftSubsystem) {
        this.m_position = position;
        this.m_liftSubsystem = liftSubsystem;
        addRequirements(m_liftSubsystem);
    }

    public void execute(){
        m_liftSubsystem.setPosition(m_position.getAsDouble());
    }

    public boolean isFinished() {
        return m_liftSubsystem.atPosition();
    }
}

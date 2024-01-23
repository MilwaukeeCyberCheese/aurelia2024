package frc.robot.commands.LiftCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LiftSubsystem;

public class ManualLiftCommand extends Command {
    public final LiftSubsystem m_liftSubsystem;
    public final DoubleSupplier m_positionAdjust;

    public ManualLiftCommand(DoubleSupplier positionAdjust, LiftSubsystem liftSubsystem) {
        this.m_positionAdjust = positionAdjust;
        this.m_liftSubsystem = liftSubsystem;
        addRequirements(m_liftSubsystem);
    }

    public void execute() {
        double position = m_liftSubsystem.getPosition()
                + m_positionAdjust.getAsDouble() * Constants.LiftConstants.kManualModifier;
        m_liftSubsystem.setPosition(position);
    }

    public boolean isFinished() {
        return m_liftSubsystem.atPosition();
    }
}

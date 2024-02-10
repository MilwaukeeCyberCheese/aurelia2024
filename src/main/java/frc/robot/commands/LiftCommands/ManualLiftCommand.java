package frc.robot.commands.LiftCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LiftSubsystem;

public class ManualLiftCommand extends Command {
    public final LiftSubsystem m_liftSubsystem;
    public final DoubleSupplier m_positionAdjust;

    /**
     * Command to jog the lift manually
     * 
     * @param adjustAmount (-1 to 1)
     * @param liftSubsystem
     */
    public ManualLiftCommand(DoubleSupplier adjustAmount, LiftSubsystem liftSubsystem) {
        this.m_positionAdjust = () -> MathUtil.clamp(adjustAmount.getAsDouble(), -1.0, 1.0);
        this.m_liftSubsystem = liftSubsystem;
        addRequirements(m_liftSubsystem);
    }

    @Override
    public void execute() {
        double position = m_liftSubsystem.getPosition()
                + m_positionAdjust.getAsDouble() * Constants.LiftConstants.kManualModifier;
        m_liftSubsystem.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return m_liftSubsystem.atPosition();
    }
}

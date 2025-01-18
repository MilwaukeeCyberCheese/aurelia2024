package frc.robot.commands.LiftCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LiftSubsystem;

public class ManualLift extends Command {
    public final LiftSubsystem m_liftSubsystem;
    public final DoubleSupplier m_positionAdjust;
    public final BooleanSupplier m_override;

    /**
     * Command to jog the lift manually
     * 
     * @param adjustAmount (-1 to 1)
     * @param liftSubsystem
     */
    public ManualLift(DoubleSupplier adjustAmount, BooleanSupplier override, LiftSubsystem liftSubsystem) {
        this.m_positionAdjust = () -> MathUtil.clamp(adjustAmount.getAsDouble(), -1.0, 1.0);
        m_override = override;
        this.m_liftSubsystem = liftSubsystem;
        addRequirements(m_liftSubsystem);
    }

    @Override
    public void execute() {
        double position = m_liftSubsystem.getPosition()
                + m_positionAdjust.getAsDouble() * Constants.LiftConstants.kManualModifier;
        m_liftSubsystem.setPosition(position, m_override.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return m_liftSubsystem.atPosition();
    }
}

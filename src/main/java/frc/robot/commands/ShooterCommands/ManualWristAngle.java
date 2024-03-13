package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualWristAngle extends Command {
    public final ShooterSubsystem m_shooterSubsystem;
    public final DoubleSupplier m_positionAdjust;

    /**
     * Command to jog the wrist manually
     * 
     * @param adjustAmount (-1 to 1)
     * @param liftSubsystem
     */
    public ManualWristAngle(DoubleSupplier adjustAmount, ShooterSubsystem shooterSubsystem) {
        this.m_positionAdjust = () -> MathUtil.clamp(adjustAmount.getAsDouble(), -1.0, 1.0);
        this.m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void execute() {
        double position = m_shooterSubsystem.getPosition()
                + m_positionAdjust.getAsDouble() * Constants.ShooterConstants.kManualModifier;
        m_shooterSubsystem.setPosition(position);
    }

    @Override
    public boolean isFinished() {
        return m_shooterSubsystem.atPosition();
    }
}

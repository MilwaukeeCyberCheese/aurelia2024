package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualAngle extends Command {
    public final ShooterSubsystem m_shooterSubsystem;
    public final DoubleSupplier m_positionAdjust;

    /**
     * Command to jog the lift manually
     * 
     * @param adjustAmount (-1 to 1)
     * @param liftSubsystem
     */
    public ManualAngle(DoubleSupplier adjustAmount, ShooterSubsystem shooterSubsystem) {
        this.m_positionAdjust = () -> MathUtil.clamp(adjustAmount.getAsDouble(), -1.0, 1.0);
        this.m_shooterSubsystem = shooterSubsystem;
        addRequirements(m_shooterSubsystem);
    }

    // @Override
    // public void execute() {
    //     double position = m_liftSubsystem.getPosition()
    //             + m_positionAdjust.getAsDouble() * Constants.LiftConstants.kManualModifier;
    //     m_liftSubsystem.setPosition(position);
    // }

    // @Override
    // public boolean isFinished() {
    //     return m_liftSubsystem.atPosition();
    // }
}

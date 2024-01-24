package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
p
public class Shoot extends Command {
    private 

    public Shoot(DoubleSupplier rpm, ShooterSubsystem shooterSubsystem) {
        addRequirements(m_intakeSubsystem, m_shooterSubsystem);
    }
}

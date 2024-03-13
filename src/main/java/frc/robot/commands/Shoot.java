package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.IntakeSpeedCommand;
import frc.robot.commands.ShooterCommands.SpinAndAngle;
import frc.robot.commands.ShooterCommands.SpinDownCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.WaitCommandMilli;

public class Shoot extends SequentialCommandGroup {
        public Shoot(DoubleSupplier rpm, DoubleSupplier shootAngle, IntakeSubsystem intakeSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        LiftSubsystem liftSubsystem) {
                addCommands(new SpinAndAngle(shootAngle, rpm, shooterSubsystem),
                                Commands.race(new IntakeSpeedCommand(() -> Constants.IntakeConstants.kLoadSpeed,
                                                intakeSubsystem),
                                                new WaitCommandMilli(Constants.ShooterConstants.kShotWaitTime)),
                                new SpinDownCommand(shooterSubsystem));
        }
}

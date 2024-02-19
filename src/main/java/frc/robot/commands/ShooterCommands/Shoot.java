package frc.robot.commands.ShooterCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.LoadCommand;
import frc.robot.commands.IntakeCommands.IntakeAngleCommand;
import frc.robot.commands.LiftCommands.LiftPositionCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.WaitCommandMilli;

public class Shoot extends SequentialCommandGroup {
        public Shoot(DoubleSupplier rpm, DoubleSupplier shootAngle, IntakeSubsystem intakeSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        LiftSubsystem liftSubsystem) {
                addCommands(
                                Commands.parallel(
                                                new LiftPositionCommand(() -> Constants.LiftConstants.kShootPosition,
                                                                liftSubsystem),
                                                new IntakeAngleCommand(
                                                                () -> Constants.IntakeConstants.kintakeAngleShootPosition,
                                                                intakeSubsystem),
                                                new SetWristAngleCommand(shootAngle,
                                                                shooterSubsystem),
                                                new SpinUpCommand(rpm, shooterSubsystem)),

                                Commands.race(new LoadCommand(intakeSubsystem),
                                                new WaitCommandMilli(Constants.ShooterConstants.kShotWaitTime)),
                                new SpinDownCommand(shooterSubsystem));
        }
}

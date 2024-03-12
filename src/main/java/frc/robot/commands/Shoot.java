package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.LoadCommand;
import frc.robot.commands.IntakeCommands.IntakePositionCommand;
import frc.robot.commands.LiftCommands.LiftPositionCommand;
import frc.robot.commands.ShooterCommands.SetWristAngleCommand;
import frc.robot.commands.ShooterCommands.SpinAndAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.WaitCommandMilli;

public class Shoot extends SequentialCommandGroup {
        public Shoot(DoubleSupplier rpm, DoubleSupplier shootAngle, IntakeSubsystem intakeSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        LiftSubsystem liftSubsystem) {
                addCommands(new SetWristAngleCommand(() -> 90, shooterSubsystem),

                                new LiftPositionCommand(() -> Constants.LiftConstants.kShootPosition,
                                                liftSubsystem),

                                new SetWristAngleCommand(() -> 150,
                                                shooterSubsystem),
                                new IntakePositionCommand(
                                                () -> Constants.IntakeConstants.kIntakeLoadPosition,
                                                intakeSubsystem),
                                new SpinAndAngle(shootAngle, rpm, shooterSubsystem),

                                Commands.race(new LoadCommand(intakeSubsystem),
                                                new WaitCommandMilli(Constants.ShooterConstants.kShotWaitTime)),
                                new SpinAndAngle(() -> 150, () -> 0, shooterSubsystem));
        }
}

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.LoadCommand;
import frc.robot.commands.IntakeCommands.IntakePositionCommand;
import frc.robot.commands.LiftCommands.LiftPositionCommand;
import frc.robot.commands.ShooterCommands.SetWristAngleCommand;
import frc.robot.commands.ShooterCommands.SpinDownCommand;
import frc.robot.commands.ShooterCommands.SpinUpCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.WaitCommandMilli;

public class ReadyToShoot extends SequentialCommandGroup {
        public ReadyToShoot(DoubleSupplier rpm, DoubleSupplier shootAngle, IntakeSubsystem intakeSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        LiftSubsystem liftSubsystem) {
                addCommands(new SpinUpCommand(rpm, shooterSubsystem),
                                Commands.parallel(
                                                new LiftPositionCommand(() -> Constants.LiftConstants.kShootPosition,
                                                                liftSubsystem),
                                                new IntakePositionCommand(
                                                                () -> Constants.IntakeConstants.kIntakeLoadPosition,
                                                                intakeSubsystem),
                                                new SetWristAngleCommand(shootAngle,
                                                                shooterSubsystem)));
        }
}

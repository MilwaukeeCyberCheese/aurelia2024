package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.SetIntakePosition;
import frc.robot.commands.LiftCommands.SetLiftPosition;
import frc.robot.commands.ShooterCommands.SetWristAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeUntilDone extends SequentialCommandGroup {
        /**
         * Overall command for intaking, then storing and centering the note
         * 
         * @param intakeSubsystem
         * @param liftSubsystem
         * @param shooterSubsystem
         */
        public IntakeUntilDone(IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem,
                        ShooterSubsystem shooterSubsystem) {
                addCommands(
                                Commands.parallel(
                                                new SetLiftPosition(() -> Constants.LiftConstants.kLoadPosition,
                                                                liftSubsystem),
                                                new SetWristAngle(
                                                                () -> Constants.ShooterConstants.kIntakeSafeAngle,
                                                                shooterSubsystem),
                                                new SetIntakePosition(
                                                                () -> Constants.IntakeConstants.kIntakeOutPosition,
                                                                intakeSubsystem)),

                                new IntakeCommand(intakeSubsystem, () -> false));
        }
}

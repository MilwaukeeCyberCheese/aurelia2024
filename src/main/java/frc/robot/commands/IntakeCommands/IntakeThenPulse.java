package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.LiftCommands.SetLiftPosition;
import frc.robot.commands.ShooterCommands.SetWristAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeThenPulse extends SequentialCommandGroup {
        /**
         * Overall command for intaking, then storing and centering the note
         * 
         * @param intakeSubsystem
         * @param liftSubsystem
         * @param shooterSubsystem
         */
        public IntakeThenPulse(IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem,
                        ShooterSubsystem shooterSubsystem) {
                addCommands(
                                Commands.parallel(new SetLiftPosition(() -> 0.0, liftSubsystem),
                                                new SetWristAngle(
                                                                () -> Constants.ShooterConstants.kIntakeSafeAngle,
                                                                shooterSubsystem)),
                                new SetIntakePosition(() -> Constants.IntakeConstants.kIntakeOutPosition,
                                                intakeSubsystem),
                                new IntakeCommand(intakeSubsystem),
                                Commands.parallel(
                                                new SetIntakePosition(
                                                                () -> Constants.IntakeConstants.kIntakeStowedPosition,
                                                                intakeSubsystem),
                                                new SetLiftPosition(() -> 0.0, liftSubsystem),
                                                new SetWristAngle(
                                                                () -> Constants.ShooterConstants.kIntakeSafeAngle,
                                                                shooterSubsystem)),
                                new SetIntakePosition(() -> Constants.IntakeConstants.kIntakeLoadPosition,
                                                intakeSubsystem),
                                new SetWristAngle(() -> 108, shooterSubsystem),
                                new Pulse(intakeSubsystem), new Pulse(intakeSubsystem), new Pulse(intakeSubsystem));
        }
}

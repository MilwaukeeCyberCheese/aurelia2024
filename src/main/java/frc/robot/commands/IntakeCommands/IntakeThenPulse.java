package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.LiftCommands.LiftPositionCommand;
import frc.robot.commands.ShooterCommands.SetWristAngleCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeThenPulse extends SequentialCommandGroup {
        /**
         * Overall command for going to the intake position, and then intaking
         * 
         * @param intakeSubsystem
         */
        public IntakeThenPulse(IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem,
                        ShooterSubsystem shooterSubsystem) {
                addCommands(
                                Commands.parallel(new LiftPositionCommand(() -> 0.0, liftSubsystem),
                                                new SetWristAngleCommand(
                                                                () -> Constants.ShooterConstants.kIntakeInAngle,
                                                                shooterSubsystem)),
                                new IntakePositionCommand(() -> Constants.IntakeConstants.kIntakeOutPosition,
                                                intakeSubsystem),
                                new IntakeCommand(intakeSubsystem),
                                Commands.parallel(
                                                new IntakePositionCommand(
                                                                () -> Constants.IntakeConstants.kIntakeStowedPosition,
                                                                intakeSubsystem),
                                                new LiftPositionCommand(() -> 0.0, liftSubsystem),
                                                new SetWristAngleCommand(
                                                                () -> Constants.ShooterConstants.kIntakeInAngle,
                                                                shooterSubsystem)),
                                new IntakePositionCommand(() -> Constants.IntakeConstants.kIntakeLoadPosition,
                                                intakeSubsystem),
                                new SetWristAngleCommand(() -> 108, shooterSubsystem),
                                new Pulse(intakeSubsystem), new Pulse(intakeSubsystem), new Pulse(intakeSubsystem));
        }
}

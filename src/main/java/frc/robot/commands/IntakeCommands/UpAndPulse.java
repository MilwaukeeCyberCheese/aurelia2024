package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.LiftCommands.SetLiftPosition;
import frc.robot.commands.ShooterCommands.SetSpinAndAngle;
import frc.robot.commands.ShooterCommands.SetWristAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class UpAndPulse extends SequentialCommandGroup {
        /**
         * Overall command for intaking, then storing and centering the note
         * 
         * @param intakeSubsystem
         * @param liftSubsystem
         * @param shooterSubsystem
         */
        public UpAndPulse(IntakeSubsystem intakeSubsystem, LiftSubsystem liftSubsystem,
                        ShooterSubsystem shooterSubsystem) {
                addCommands(

                                Commands.parallel(
                                                new SetIntakePosition(
                                                                () -> Constants.IntakeConstants.kIntakeLoadPosition,
                                                                intakeSubsystem),
                                                new SetLiftPosition(() -> Constants.LiftConstants.kLoadPosition,
                                                                liftSubsystem),
                                                new SetWristAngle(
                                                                () -> Constants.ShooterConstants.kIntakeSafeAngle,
                                                                shooterSubsystem)),

                                new SetWristAngle(() -> Constants.ShooterConstants.kShootAngle, shooterSubsystem),
                                new Pulse(intakeSubsystem),
                                new SetSpinAndAngle(() -> 70, () -> 2000, () -> 2000, shooterSubsystem).onlyIf(Robot.m_autoSpin::getSelected));
        }
}

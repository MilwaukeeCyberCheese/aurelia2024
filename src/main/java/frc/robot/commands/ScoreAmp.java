package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.SetIntakePosition;
import frc.robot.commands.LiftCommands.SetLiftPosition;
import frc.robot.commands.ShooterCommands.SetSpin;
import frc.robot.commands.ShooterCommands.SetSpinAndAngle;
import frc.robot.commands.ShooterCommands.SetWristAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.WaitCommandMilli;

public class ScoreAmp extends SequentialCommandGroup {
    public ScoreAmp(ShooterSubsystem shooterSubsystem, LiftSubsystem liftSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(new SetWristAngle(() -> Constants.ShooterConstants.kIntakeSafeAngle, shooterSubsystem),
                new SetIntakePosition(() -> Constants.IntakeConstants.kIntakeStowedPosition, intakeSubsystem),
                new SetWristAngle(() -> Constants.ShooterConstants.kLiftSafeAngle, shooterSubsystem),
                new SetLiftPosition(() -> Constants.LiftConstants.kAmpPosition, liftSubsystem),
                new SetWristAngle(() -> Constants.ShooterConstants.kAmpAngle, shooterSubsystem),
                Commands.parallel(new SetSpin(() -> Constants.ShooterConstants.kAmpRPM, shooterSubsystem),
                        new WaitCommandMilli(Constants.ShooterConstants.kAmpWaitTime)),
                new SetSpinAndAngle(() -> Constants.ShooterConstants.kLiftSafeAngle, () -> 0, shooterSubsystem),
                new SetLiftPosition(() -> Constants.LiftConstants.kLoadPosition, liftSubsystem),
                new SetWristAngle(() -> Constants.ShooterConstants.kIntakeSafeAngle, shooterSubsystem),
                new SetIntakePosition(() -> Constants.IntakeConstants.kIntakeLoadPosition, intakeSubsystem));
    }
}

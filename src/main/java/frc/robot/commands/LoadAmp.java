package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.SetIntakeSpeed;
import frc.robot.commands.ShooterCommands.SetSpin;
import frc.robot.commands.ShooterCommands.SetSpinAndAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.WaitCommandMilli;

public class LoadAmp extends SequentialCommandGroup {
    public LoadAmp(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(new SetSpinAndAngle(() -> Constants.ShooterConstants.kHandoffAngle,
                () -> Constants.ShooterConstants.kLoadRPM, () -> Constants.ShooterConstants.kLoadRPM,
                shooterSubsystem),
                Commands.race(new SetIntakeSpeed(() -> Constants.IntakeConstants.kHandoffSpeed, intakeSubsystem),
                        new WaitCommandMilli(Constants.IntakeConstants.kHandoffTime), Commands.parallel(
                                new SetIntakeSpeed(() -> 0.0, intakeSubsystem), new SetSpin(() -> 0.0, shooterSubsystem))));
    }
}

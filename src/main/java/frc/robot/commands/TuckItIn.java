package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.SetIntakePosition;
import frc.robot.commands.LiftCommands.SetLiftPosition;
import frc.robot.commands.ShooterCommands.SetWristAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TuckItIn extends SequentialCommandGroup {
    public TuckItIn(LiftSubsystem liftSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(new SetLiftPosition(() -> Constants.LiftConstants.kLoadPosition, liftSubsystem), new SetWristAngle(() -> Constants.ShooterConstants.kIntakeSafeAngle, shooterSubsystem), new SetIntakePosition(() -> Constants.IntakeConstants.kIntakeLoadPosition, intakeSubsystem));
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.SetIntakePosition;
import frc.robot.commands.IntakeCommands.SetIntakeSpeed;
import frc.robot.commands.ShooterCommands.SetWristAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreAmpIntake extends SequentialCommandGroup {
    /**
     * Command to score in the amp using the intake
     * 
     * @param shooterSubsystem
     * @param intakeSubsystem
     */
    public ScoreAmpIntake(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
                new SetWristAngle(() -> Constants.ShooterConstants.kIntakeSafeAngle, shooterSubsystem),
                new SetIntakePosition(() -> Constants.IntakeConstants.kIntakeAmpPosition, intakeSubsystem),
                new SetIntakeSpeed(() -> Constants.IntakeConstants.kAmpSpeed, intakeSubsystem));
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.IntakeFromGround;
import frc.robot.subsystems.IntakeCameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class FollowAndIntake extends SequentialCommandGroup {
    public FollowAndIntake(IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem,
            IntakeCameraSubsystem intakeCameraSubsystem) {
        addCommands(
                new FollowNote(driveSubsystem, intakeCameraSubsystem, () -> Constants.IntakeConstants.kDeployRange),
                Commands.race(new FollowNote(driveSubsystem, intakeCameraSubsystem, () -> -1.0),
                        new IntakeFromGround(intakeSubsystem)));
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.IntakeFromGround;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class FollowAndIntake extends SequentialCommandGroup {
public FollowAndIntake(IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem){
        addCommands(
            Commands.race(new FollowTarget(driveSubsystem, cameraSubsystem, () -> 0), new IntakeFromGround(intakeSubsystem))
        );
    }
}

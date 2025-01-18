package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.IntakeThenPulse;
import frc.robot.subsystems.IntakeCameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FollowAndIntake extends SequentialCommandGroup {
    /**
     * Follow a note using vision and intake it
     * 
     * @param intakeSubsystem
     * @param driveSubsystem
     * @param intakeCameraSubsystem
     * @param liftSubsystem
     * @param shooterSubsystem
     */
    public FollowAndIntake(IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem,
            IntakeCameraSubsystem intakeCameraSubsystem, LiftSubsystem liftSubsystem,
            ShooterSubsystem shooterSubsystem) {
        addCommands(
                Commands.race(new FollowNote(driveSubsystem, intakeCameraSubsystem, () -> -1.0),
                        new IntakeThenPulse(intakeSubsystem, liftSubsystem, shooterSubsystem, () -> false)));
    }
}

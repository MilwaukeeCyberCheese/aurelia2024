package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.LiftCommands.LiftPositionCommand;
import frc.robot.commands.ShooterCommands.AmpShooter;
import frc.robot.commands.ShooterCommands.SetWristAngleCommand;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.WaitCommandMilli;;

public class ScoreAmpCommand extends SequentialCommandGroup {

    public ScoreAmpCommand(ShooterSubsystem shooterSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(
                Commands.parallel(new LiftPositionCommand(() -> Constants.LiftConstants.kAmpPosition, liftSubsystem),
                        new SetWristAngleCommand(() -> Constants.ShooterConstants.kAmpAngle, shooterSubsystem)),
                Commands.race(new AmpShooter(shooterSubsystem), new WaitCommandMilli(Constants.ShooterConstants.kShotWaitTime)));
    }

}

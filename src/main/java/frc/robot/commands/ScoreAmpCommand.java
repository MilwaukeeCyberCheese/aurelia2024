package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.LiftCommands.LiftPositionCommand;
import frc.robot.commands.ShooterCommands.AmpShooter;
import frc.robot.commands.ShooterCommands.SetShooterAngleCommand;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ScoreAmpCommand extends SequentialCommandGroup {

    public ScoreAmpCommand(ShooterSubsystem shooterSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(new LiftPositionCommand(() -> Constants.LiftConstants.kAmpPosition, liftSubsystem),
                new SetShooterAngleCommand(() -> Constants.ShooterConstants.kAmpAngle, shooterSubsystem),
                new AmpShooter(shooterSubsystem));
    }

}

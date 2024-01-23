package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.LiftCommands.ManualLiftCommand;
import frc.robot.commands.ShooterCommands.SetShooterAngleCommand;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpCommand extends SequentialCommandGroup {

    public AmpCommand(ShooterSubsystem shooterSubsystem, LiftSubsystem liftSubsystem) {
        addCommands(new SetShooterAngleCommand(Constants.ShooterConstants.kStowAngle, shooterSubsystem),
                new ManualLiftCommand(Constants.LiftConstants.kAmpPosition, liftSubsystem));
    }

}

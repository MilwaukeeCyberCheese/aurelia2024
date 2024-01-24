package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.LoadCommand;
import frc.robot.commands.IntakeCommands.PivotCommand;
import frc.robot.commands.LiftCommands.LiftPositionCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LoadShooter extends SequentialCommandGroup {
    public LoadShooter(ShooterSubsystem shooterSubsystem, LiftSubsystem liftSubsystem,
            IntakeSubsystem intakeSubsystem) {
        addCommands(new LiftPositionCommand(() -> Constants.LiftConstants.kLoadPosition, liftSubsystem),
                new PivotCommand(() -> Constants.IntakeConstants.kPivotLoadPosition, intakeSubsystem),
                new LoadCommand(intakeSubsystem));
    }
}

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.LoadCommand;
import frc.robot.commands.IntakeCommands.IntakeAngleCommand;
import frc.robot.commands.LiftCommands.LiftPositionCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LoadShooter extends SequentialCommandGroup {
        /**
         * Command to load the note into the shooter from the intake
         * 
         * @param shooterSubsystem
         * @param liftSubsystem
         * @param intakeSubsystem
         */
        public LoadShooter(ShooterSubsystem shooterSubsystem, LiftSubsystem liftSubsystem,
                        IntakeSubsystem intakeSubsystem) {
                addCommands(
                                Commands.parallel(
                                                new LiftPositionCommand(() -> Constants.LiftConstants.kLoadPosition,
                                                                liftSubsystem),
                                                new IntakeAngleCommand(
                                                                () -> Constants.IntakeConstants.kintakeAngleLoadPosition,
                                                                intakeSubsystem),
                                                new SetWristAngleCommand(() -> Constants.ShooterConstants.kLoadAngle,
                                                                shooterSubsystem)),
                                Commands.race(
                                                new LoadCommand(intakeSubsystem),
                                                new LoadShooterCommand(shooterSubsystem)));
        }
}

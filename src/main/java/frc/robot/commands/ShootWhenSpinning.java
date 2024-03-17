package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.SetIntakeSpeed;
import frc.robot.commands.ShooterCommands.SetSpin;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.WaitCommandMilli;

public class ShootWhenSpinning extends SequentialCommandGroup {
        /**
         * Command to shoot to the speaker
         * 
         * @param intakeSubsystem
         * @param shooterSubsystem
         * @param liftSubsystem
         */
        public ShootWhenSpinning(
                        IntakeSubsystem intakeSubsystem,
                        ShooterSubsystem shooterSubsystem,
                        LiftSubsystem liftSubsystem) {
                addCommands(
                                Commands.race(new SetIntakeSpeed(() -> Constants.IntakeConstants.kOuttakeSpeed,
                                                intakeSubsystem),
                                                new WaitCommandMilli(Constants.ShooterConstants.kShotWaitTime)),
                                new SetSpin(() -> 0, shooterSubsystem), new Command() {
                                        @Override
                                        public void execute() {
                                                SmartDashboard.putBoolean("Shoot", false);
                                        }
                                });
        }
}

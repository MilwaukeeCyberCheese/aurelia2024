package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeFromGround extends SequentialCommandGroup{
    public IntakeFromGround(IntakeSubsystem intakeSubsystem){
        addCommands(
            new GroundCommand(intakeSubsystem),
            new IntakeCommand(intakeSubsystem)
        );
    }
}

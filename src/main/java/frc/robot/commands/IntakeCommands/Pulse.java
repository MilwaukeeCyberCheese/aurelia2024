package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.WaitCommandMilli;

public class Pulse extends SequentialCommandGroup {
    private double speed = 0.2;
    private int waitTime = 100;

    public Pulse(IntakeSubsystem m_IntakeSubsystem) {
        addCommands(Commands.race(new IntakeSpeedCommand(() -> speed, m_IntakeSubsystem),
                new WaitCommandMilli(waitTime)), 
                Commands.race(new IntakeSpeedCommand(() -> -speed, m_IntakeSubsystem),
                new WaitCommandMilli(waitTime)), Commands.race(new IntakeSpeedCommand(() -> speed, m_IntakeSubsystem),
                new WaitCommandMilli(waitTime)), 
                Commands.race(new IntakeSpeedCommand(() -> -speed, m_IntakeSubsystem),
                new WaitCommandMilli(waitTime)), Commands.race(new IntakeSpeedCommand(() -> speed, m_IntakeSubsystem),
                new WaitCommandMilli(waitTime)), 
                Commands.race(new IntakeSpeedCommand(() -> -speed, m_IntakeSubsystem),
                new WaitCommandMilli(waitTime)), Commands.race(new IntakeSpeedCommand(() -> speed, m_IntakeSubsystem),
                new WaitCommandMilli(waitTime))
                );
    }
}
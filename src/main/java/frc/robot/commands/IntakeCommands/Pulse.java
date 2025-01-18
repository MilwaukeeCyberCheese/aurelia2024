package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.utils.WaitCommandMilli;

public class Pulse extends SequentialCommandGroup {
        private double speed = Constants.IntakeConstants.kPulseSpeed;
        private double waitTime = Constants.IntakeConstants.kPulseTime;

        /**
         * Command to pulse the intake, centering the note
         * 
         * @param m_IntakeSubsystem
         */
        public Pulse(IntakeSubsystem m_IntakeSubsystem) {
                addCommands(Commands.race(new SetIntakeSpeed(() -> speed, m_IntakeSubsystem),
                                new WaitCommandMilli(waitTime)),
                                Commands.race(new SetIntakeSpeed(() -> -speed, m_IntakeSubsystem),
                                                new WaitCommandMilli(waitTime)),
                                Commands.race(new SetIntakeSpeed(() -> speed, m_IntakeSubsystem),
                                                new WaitCommandMilli(waitTime)),
                                Commands.race(new SetIntakeSpeed(() -> -speed, m_IntakeSubsystem),
                                                new WaitCommandMilli(waitTime)),
                                Commands.race(new SetIntakeSpeed(() -> speed, m_IntakeSubsystem),
                                                new WaitCommandMilli(waitTime)),
                                Commands.race(new SetIntakeSpeed(() -> -speed, m_IntakeSubsystem),
                                                new WaitCommandMilli(waitTime)),
                                Commands.race(new SetIntakeSpeed(() -> speed, m_IntakeSubsystem),
                                                new WaitCommandMilli(waitTime)));
        }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStop extends Command {
    private final DriveSubsystem m_driveSubsystem;

    /**
     * Stops the drivetrain from moving
     * 
     * @param driveSubsystem subsystem for driving the robot {@link frc.robot.subsystems.DriveSubsystem link}
     */
    public DriveStop(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        m_driveSubsystem.drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
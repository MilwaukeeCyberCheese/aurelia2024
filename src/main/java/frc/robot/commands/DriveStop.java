package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveStop extends Command {
    private final DriveSubsystem m_driveSubsystem;

    /**
     * Stops the drivetrain from moving
     * 
     * @param driveSubsystem
     */
    public DriveStop(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        m_driveSubsystem.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class OrientToTarget extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final CameraSubsystem m_cameraSubsytem;

    /**
     * 
     * @param driveSubsystem  subsystem for driving the robot
     *                        {@link frc.robot.subsystems.DriveSubsystem link}
     * @param cameraSubsystem subsystem containing the cameras
     *                        {@link frc.robot.subsystems.CameraSubsystem link}
     */
    public OrientToTarget(DriveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_cameraSubsytem = cameraSubsystem;
        addRequirements(m_cameraSubsytem, m_driveSubsystem);
    }

    @Override
    public void execute() {
        double thetaOutput = 0;
        PhotonTrackedTarget target = m_cameraSubsytem.getRightTarget();
        if (target != null) {
            thetaOutput = target.getYaw();
        }

        m_cameraSubsytem.logging(thetaOutput, 0, 0);

        m_driveSubsystem.drive(new Pose2d(0, 0, new Rotation2d(thetaOutput)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

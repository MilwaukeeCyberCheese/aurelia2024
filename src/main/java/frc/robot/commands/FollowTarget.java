package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTarget extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final CameraSubsystem m_cameraSubsytem;

    /**
     * Point towards, and move towards, a detected
     * {@link org.photonvision.targeting.PhotonTrackedTarget#PhotonTrackedTarget()
     * PhotonTrackedTarget}
     * 
     * @param driveSubsystem  subsystem used for driving
     *                        {@link frc.robot.subsystems.DriveSubsystem link}
     * @param cameraSubsystem subsystem containing the cameras
     *                        {@link frc.robot.subsystems.CameraSubsystem link}
     */
    public FollowTarget(DriveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_cameraSubsytem = cameraSubsystem;
        addRequirements(m_cameraSubsytem, m_driveSubsystem);
    }

    @Override
    public void execute() {
        double thetaOutput = 0;
        double yOutput = 0;
        double range = 0;
        PhotonTrackedTarget target = m_cameraSubsytem.getRightTarget();

        //check if target is present
        if (target != null) {
            //set theta based on yaw
            thetaOutput = Math.toRadians(target.getYaw());

            //calculate range
            range = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.VisionConstants.kCameraHeight,
                    Constants.VisionConstants.kCubeHeight,
                    0,
                    Units.degreesToRadians(target.getPitch()));

            //set y based on range
            yOutput = (range > 0.2) ? range - 0.5 : 0.0;

        }

        m_driveSubsystem.drive(new Pose2d(0.0, yOutput, new Rotation2d(thetaOutput)));

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
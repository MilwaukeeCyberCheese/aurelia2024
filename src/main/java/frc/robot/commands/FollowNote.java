package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterCameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class FollowNote extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final ShooterCameraSubsystem m_cameraSubsytem;
    private final DoubleSupplier m_goalRange;

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
    public FollowNote(DriveSubsystem driveSubsystem, ShooterCameraSubsystem cameraSubsystem, DoubleSupplier goalRange) {
        m_driveSubsystem = driveSubsystem;
        m_cameraSubsytem = cameraSubsystem;
        m_goalRange = goalRange;
        addRequirements(m_cameraSubsytem, m_driveSubsystem);
    }

    @Override
    public void execute() {
        double thetaOutput = 0;
        //TODO: like everything here needs a revamp
        double yOutput = 0;
        double range = 0;
        PhotonTrackedTarget target = m_cameraSubsytem.getTarget();

        //check if target is present
        if (target != null) {
            //set theta based on yaw
            thetaOutput = Math.toRadians(target.getYaw());
            

            //calculate range
            range = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.IntakeCamera.kCameraHeight,
                    Constants.IntakeCamera.kCameraHeight, //TODO: replace with height of note
                    0,
                    Units.degreesToRadians(target.getPitch()));

            //set y based on range
            yOutput = (range > m_goalRange.getAsDouble()) ? range - m_goalRange.getAsDouble() : 0.0;

        }

        m_driveSubsystem.drive(new Pose2d(0.0, yOutput, new Rotation2d(thetaOutput)));

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
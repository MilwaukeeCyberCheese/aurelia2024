package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeCameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class FollowNote extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final IntakeCameraSubsystem m_cameraSubsytem;
    private final DoubleSupplier m_goalRange;
    private double range = 0;

    /**
     * Point towards, and move towards, a detected
     * {@link org.photonvision.targeting.PhotonTrackedTarget#PhotonTrackedTarget()
     * PhotonTrackedTarget}
     * 
     * @param driveSubsystem  subsystem used for driving
     * @param cameraSubsystem subsystem containing the camera
     * @param goalRange       the range at which the robot should stop (meters)
     */
    public FollowNote(DriveSubsystem driveSubsystem, IntakeCameraSubsystem cameraSubsystem, DoubleSupplier goalRange) {
        m_driveSubsystem = driveSubsystem;
        m_cameraSubsytem = cameraSubsystem;
        m_goalRange = goalRange;
        addRequirements(m_cameraSubsytem, m_driveSubsystem);
    }

    @Override
    public void execute() {
        double thetaOutput = 0;
        // TODO: like everything here needs a revamp, maybe use PPLib?
        double yOutput = 0;
        PhotonTrackedTarget target = m_cameraSubsytem.getTarget();

        // check if target is present
        if (target != null) {
            // set theta based on yaw
            thetaOutput = Math.toRadians(target.getYaw());

            // calculate range
            range = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.VisionConstants.IntakeCamera.kCameraHeight,
                    Constants.VisionConstants.Note.kHeight,
                    Constants.VisionConstants.IntakeCamera.kRobotToCam.getRotation().getY(),
                    Units.degreesToRadians(target.getPitch()));

            // set y based on range
            yOutput = (range > m_goalRange.getAsDouble()) ? range - m_goalRange.getAsDouble() : 0.0;

        }
        SmartDashboard.putNumber("Range to Note", yOutput);

        m_driveSubsystem.drive(new Pose2d(0.0, yOutput, new Rotation2d(thetaOutput)));

    }

    @Override
    public boolean isFinished() {
        return m_goalRange.getAsDouble() >= range;
    }
}
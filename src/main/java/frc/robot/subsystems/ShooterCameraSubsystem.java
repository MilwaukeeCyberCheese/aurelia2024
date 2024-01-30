//TODO make this prettier
package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterCameraSubsystem extends SubsystemBase {
    private PhotonTrackedTarget target;

    // TODO: rework and revamp
    /**
     * Has all of the functions necessary for working with the shooter camera
     */
    public ShooterCameraSubsystem() {

    }

    /**
     * switches pipeline of a camera
     * 
     * @param index  pipeline to switch to
     * @param camera camera to switch pipelines for
     */
    public static void switchIndex(int index, int camera) {

        Constants.VisionConstants.ShooterCamera.kCamera.setPipelineIndex(index);

    }

    /**
     * Whether the camera is in driver mode, or running the pipeline
     * 
     * @param mode
     */
    public static void setDriverMode(boolean mode) {
        Constants.VisionConstants.ShooterCamera.kCamera.setDriverMode(mode);
    }

    /**
     * Before using the target, ensure that it is present
     * 
     * @return the target found by the shooter camera
     * 
     */
    public PhotonTrackedTarget getTarget() {

        var result = Constants.VisionConstants.ShooterCamera.kCamera.getLatestResult();
        target = result.getBestTarget();
        return target;

    }

    public void log() {
    }

    @Override
    public void periodic() {
        log();
        updateOdometry();
    }

    /**
     * update the odometry based on aprilTags
     */
    public void updateOdometry() {
        Constants.VisionConstants.Pose.kPoseEstimator
                .setReferencePose(Constants.DriveConstants.m_odometry.getEstimatedPosition());

        var result = Constants.VisionConstants.Pose.kPoseEstimator.update();

        if (result.isPresent()) {
            EstimatedRobotPose estimate = result.get();
            Constants.DriveConstants.m_odometry.addVisionMeasurement(estimate.estimatedPose.toPose2d(),
                    estimate.timestampSeconds);

        }
    }

}

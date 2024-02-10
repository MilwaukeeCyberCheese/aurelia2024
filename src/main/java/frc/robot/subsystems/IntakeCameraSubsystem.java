//TODO make this prettier
package frc.robot.subsystems;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeCameraSubsystem extends SubsystemBase {
    private PhotonTrackedTarget target;

    // TODO: rework and revamp
    /**
     * Has all of the functions necessary for working with the intake camera
     */
    public IntakeCameraSubsystem() {

    }

    /**
     * switches pipeline of intake camera
     * 
     * @param index pipeline to switch to
     */
    public static void switchIndex(int index) {
        Constants.VisionConstants.IntakeCamera.kCamera.setPipelineIndex(index);
    }

 /**
     * Whether the camera is in driver mode, or running the pipeline
     * 
     * @param mode
     */
    public static void setDriverMode(boolean mode) {
        Constants.VisionConstants.IntakeCamera.kCamera.setDriverMode(mode);
    }


    /**
     * Before using the target, ensure that it is present
     * 
     * @return the target found by the camera
     * 
     */
    public PhotonTrackedTarget getTarget() {

        var result = Constants.VisionConstants.IntakeCamera.kCamera.getLatestResult();
        target = result.getBestTarget();
        return target;

    }

    public void log() {
    }

    @Override
    public void periodic() {
        log();
    }

}

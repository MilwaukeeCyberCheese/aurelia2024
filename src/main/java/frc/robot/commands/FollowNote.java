package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private int counter;
    private PhotonTrackedTarget target;
    private double goalYaw;
    private double range = 0;
    private PIDController thetaController = new PIDController(Constants.AutoConstants.kThetaPIDConstants.kP,
            Constants.AutoConstants.kThetaPIDConstants.kI, Constants.AutoConstants.kThetaPIDConstants.kD);
    private PIDController yController = new PIDController(Constants.AutoConstants.kTranslationPIDConstants.kP,
            Constants.AutoConstants.kTranslationPIDConstants.kI, Constants.AutoConstants.kTranslationPIDConstants.kD);

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
    public void initialize() {
        yController.reset();
        yController.setSetpoint(m_goalRange.getAsDouble());

        thetaController.reset();
    }

    @Override
    public void execute() {
        double thetaOutput = 0;
        double yOutput = 0;

        if (counter % 5 == 0) {
            target = m_cameraSubsytem.getTarget();
            if (target != null) {
                // set theta based on yaw
                goalYaw = Math.toRadians(target.getYaw() + Constants.Sensors.gyro.getAngle());
                range = PhotonUtils.calculateDistanceToTargetMeters(
                        Constants.VisionConstants.IntakeCamera.kCameraHeight,
                        Constants.VisionConstants.Note.kHeight,
                        Constants.VisionConstants.IntakeCamera.kRobotToCam.getRotation().getY(),
                        Units.degreesToRadians(target.getPitch()));
            }
        }

        // check if target is present
        if (target != null) {
            // set theta based on yaw
            thetaOutput = thetaController.calculate(Constants.Sensors.gyro.getAngle(), goalYaw);

            // set y based on range
            yOutput = yController.calculate(range);
        }
        SmartDashboard.putNumber("Range to Note", yOutput);

        m_driveSubsystem.drive(new ChassisSpeeds(0.0, yOutput, thetaOutput));

    }

    @Override
    public boolean isFinished() {
        return m_goalRange.getAsDouble() >= range;
    }
}
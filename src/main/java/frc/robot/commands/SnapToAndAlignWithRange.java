package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterCameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SnapToAndAlignWithRange extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final ShooterCameraSubsystem m_cameraSubsytem;
    private final IntSupplier m_id;
    private final DoubleSupplier m_angle;
    private final DoubleSupplier m_goalRange;
    private PIDController m_xController = new PIDController(Constants.AutoConstants.kTranslationPIDConstants.kP,
            Constants.AutoConstants.kTranslationPIDConstants.kI, Constants.AutoConstants.kTranslationPIDConstants.kD);
    private PIDController m_yController = new PIDController(Constants.AutoConstants.kTranslationPIDConstants.kP,
            Constants.AutoConstants.kTranslationPIDConstants.kI, Constants.AutoConstants.kTranslationPIDConstants.kD);
    private PIDController m_thetaController = new PIDController(Constants.AutoConstants.kThetaPIDConstants.kP,
            Constants.AutoConstants.kThetaPIDConstants.kI, Constants.AutoConstants.kThetaPIDConstants.kD);

    /**
     * Snap to a certain angle relative to the field, and then line up with
     * the apriltag indicated in the x and y plane
     * 
     * @param driveSubsystem
     * @param cameraSubsystem
     * @param id              id of the apriltag to orient to
     * @param angle           angle to snap to on the field (degrees)
     * @param goalRange       range to achieve from the apriltag
     */
    public SnapToAndAlignWithRange(DriveSubsystem driveSubsystem, ShooterCameraSubsystem cameraSubsystem,
            IntSupplier id, DoubleSupplier angle, DoubleSupplier goalRange) {
        m_driveSubsystem = driveSubsystem;
        m_cameraSubsytem = cameraSubsystem;
        m_id = id;
        m_angle = angle;
        m_goalRange = goalRange;
        addRequirements(m_cameraSubsytem, m_driveSubsystem);
    }

    @Override
    public void initialize() {
        m_xController.reset();
        m_xController.setSetpoint(0.0);

        m_yController.reset();
        m_yController.setSetpoint(m_goalRange.getAsDouble());

        m_thetaController.reset();
        m_thetaController.setSetpoint(Math.toRadians(m_angle.getAsDouble()));
    }

    @Override
    public void execute() {
        double xOutput = 0;
        double yOutput = 0;
        double thetaOutput = 0;
        // TODO: like everything here needs a revamp, maybe use PPLib?

        PhotonTrackedTarget target = m_cameraSubsytem.getAprilTag(m_id.getAsInt());

        // check if target is present
        if (target != null) {
            // set x based on yaw
            xOutput = m_xController.calculate(target.getYaw() * -1.0);

            // set y based on range
            double range = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.VisionConstants.ShooterCamera.kCameraHeight,
                    Constants.VisionConstants.TagPositions.kTagPositions.get(m_id.getAsInt()).getZ(),
                    Constants.VisionConstants.ShooterCamera.kRobotToCam.getRotation().getY(),
                    Units.degreesToRadians(target.getPitch()));

            yOutput = m_yController.calculate(range);
        }

        //snap to theta using gyro
        thetaOutput = m_thetaController.calculate(Math.toRadians(Constants.Sensors.gyro.getAngle()));
        System.out.println("Range: " + yOutput + "Translation: " + xOutput);
        m_driveSubsystem.driveLimited(new ChassisSpeeds(-1 * yOutput, xOutput, thetaOutput));

    }

    @Override
    public boolean isFinished() {
        return m_xController.atSetpoint();
    }
}
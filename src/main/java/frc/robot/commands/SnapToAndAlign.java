package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterCameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class SnapToAndAlign extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final ShooterCameraSubsystem m_cameraSubsytem;
    private final IntSupplier m_id;
    private final DoubleSupplier m_angle;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_xSpeed;
    private PIDController m_xController = new PIDController(Constants.AutoConstants.kTranslationPIDConstants.kP,
            Constants.AutoConstants.kTranslationPIDConstants.kI, Constants.AutoConstants.kTranslationPIDConstants.kD);
    private PIDController m_thetaController = new PIDController(Constants.AutoConstants.kThetaPIDConstants.kP,
            Constants.AutoConstants.kThetaPIDConstants.kI, Constants.AutoConstants.kThetaPIDConstants.kD);

    /**
     * Snap to a certain angle relative to the field, and then line up laterally with
     * the apriltag indicated
     * 
     * @param driveSubsystem
     * @param cameraSubsystem
     * @param id              id of the apriltag to line up with
     * @param angle           angle to snap to
     * @param speed           speed to move forwards and backwards (robot relative)
     */
    public SnapToAndAlign(DriveSubsystem driveSubsystem, ShooterCameraSubsystem cameraSubsystem, IntSupplier id,
            DoubleSupplier angle, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        m_driveSubsystem = driveSubsystem;
        m_cameraSubsytem = cameraSubsystem;
        m_id = id;
        m_angle = angle;
        m_xSpeed = ySpeed;
        m_ySpeed = ySpeed;
        addRequirements(m_cameraSubsytem, m_driveSubsystem);
    }

    @Override
    public void initialize() {
        m_xController.reset();
        m_xController.setSetpoint(0.0);

        m_thetaController.reset();
        m_thetaController.setSetpoint(Math.toRadians(m_angle.getAsDouble()));
        m_thetaController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void execute() {
        double xOutput = 0;
        double thetaOutput = 0;
        // TODO: like everything here needs a revamp, maybe use PPLib?

        PhotonTrackedTarget target = m_cameraSubsytem.getAprilTag(m_id.getAsInt());

        // check if target is present
        if (target != null) {
            // set theta based on yaw
            xOutput = m_xController.calculate(target.getYaw());

        } else{
            xOutput = m_xSpeed.getAsDouble();
        }

        thetaOutput = m_thetaController.calculate(Math.toRadians(Constants.Sensors.gyro.getYaw()));
        System.out.println("Translation: " + xOutput + "Rotation: " + thetaOutput);
        m_driveSubsystem.driveLimited(new ChassisSpeeds(xOutput, m_ySpeed.getAsDouble(), thetaOutput));

    }

    @Override
    public boolean isFinished() {
        return m_xController.atSetpoint();
    }
}
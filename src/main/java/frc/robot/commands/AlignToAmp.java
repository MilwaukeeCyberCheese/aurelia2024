package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterCameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToAmp extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final ShooterCameraSubsystem m_cameraSubsytem;
    private PIDController m_xController = new PIDController(Constants.AutoConstants.kTranslationPIDConstants.kP,
            Constants.AutoConstants.kTranslationPIDConstants.kI, Constants.AutoConstants.kTranslationPIDConstants.kD);
    private PIDController m_thetaController = new PIDController(Constants.AutoConstants.kThetaPIDConstants.kP,
            Constants.AutoConstants.kThetaPIDConstants.kI, Constants.AutoConstants.kThetaPIDConstants.kD);

    /**
     * Point towards, and move towards, a detected
     * {@link org.photonvision.targeting.PhotonTrackedTarget#PhotonTrackedTarget()
     * PhotonTrackedTarget}
     * 
     * @param driveSubsystem  subsystem used for driving
     * @param cameraSubsystem subsystem containing the camera
     * @param goalRange       the range at which the robot should stop (meters)
     */
    public AlignToAmp(DriveSubsystem driveSubsystem, ShooterCameraSubsystem cameraSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_cameraSubsytem = cameraSubsystem;
        addRequirements(m_cameraSubsytem, m_driveSubsystem);
    }

    @Override
    public void initialize() {
        m_xController.reset();
        m_xController.setSetpoint(0.0);

        m_thetaController.reset();
        m_thetaController.setSetpoint(90.0);
    }

    @Override
    public void execute() {
        double xOutput = 0;
        double thetaOutput = 0;
        // TODO: like everything here needs a revamp, maybe use PPLib?

        PhotonTrackedTarget target = m_cameraSubsytem.getTarget();

        // check if target is present
        if (target != null && target.getFiducialId() == ((Robot.allianceColor) ? 5 : 6)) {
            // set theta based on yaw
            xOutput = m_xController.calculate(Math.toRadians(target.getYaw() * -1.0));

        }

        thetaOutput = m_thetaController.calculate(Math.toRadians(Constants.Sensors.gyro.getAngle()));

        m_driveSubsystem.driveLimited(new ChassisSpeeds(xOutput, 0.0, thetaOutput));

    }

    @Override
    public boolean isFinished() {
        return m_xController.atSetpoint();
    }
}
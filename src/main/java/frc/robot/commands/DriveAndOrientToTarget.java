package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeCameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAndOrientToTarget extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final IntakeCameraSubsystem m_cameraSubsytem;
    private final DoubleSupplier m_goalRange;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final BooleanSupplier m_fieldRelative;
    private final BooleanSupplier m_rateLimit;
    private double range = 0;
    private PIDController thetaController = new PIDController(Constants.AutoConstants.kThetaPIDConstants.kP,
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
    public DriveAndOrientToTarget(DriveSubsystem driveSubsystem, IntakeCameraSubsystem cameraSubsystem,
            DoubleSupplier goalRange, DoubleSupplier xSpeed, DoubleSupplier ySpeed, BooleanSupplier fieldRelative,
            BooleanSupplier rateLimit, BooleanSupplier slow, DoubleSupplier throttle) {
        m_driveSubsystem = driveSubsystem;
        m_cameraSubsytem = cameraSubsystem;
        m_goalRange = goalRange;

        if (slow.getAsBoolean()) {
            m_xSpeed = () -> xSpeed.getAsDouble() * Constants.DriveConstants.kSlowModifier;
            m_ySpeed = () -> ySpeed.getAsDouble() * Constants.DriveConstants.kSlowModifier;
        } else {
            m_xSpeed = () -> xSpeed.getAsDouble() * MathUtil.clamp(throttle.getAsDouble(), 0.1, 1.0);
            m_ySpeed = () -> ySpeed.getAsDouble() * MathUtil.clamp(throttle.getAsDouble(), 0.1, 1.0);
        }

        m_fieldRelative = fieldRelative;
        m_rateLimit = rateLimit;
        addRequirements(m_cameraSubsytem, m_driveSubsystem);
    }

    @Override
    public void execute() {
        double thetaOutput = 0;
        PhotonTrackedTarget target = m_cameraSubsytem.getTarget();

        // check if target is present
        if (target != null) {
            // set theta based on yaw
            thetaOutput = thetaController.calculate(0/* TODO: gyro */, Math.toRadians(target.getYaw() * -1.0));
        }

        m_driveSubsystem.drive(m_xSpeed.getAsDouble(), m_ySpeed.getAsDouble(), thetaOutput,
                m_fieldRelative.getAsBoolean(),
                m_rateLimit.getAsBoolean());

    }

    @Override
    public boolean isFinished() {
        return m_goalRange.getAsDouble() >= range;
    }
}
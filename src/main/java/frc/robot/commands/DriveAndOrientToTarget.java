package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeCameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAndOrientToTarget extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final IntakeCameraSubsystem m_cameraSubsytem;
    private final DoubleSupplier m_xSpeed;
    private final DoubleSupplier m_ySpeed;
    private final DoubleSupplier m_rot;
    private final BooleanSupplier m_fieldRelative;
    private final BooleanSupplier m_rateLimit;
    private PhotonTrackedTarget target;
    private int counter = 0;
    private Double goalYaw = null;
    private Double thetaOutput = 0.0;
    private PIDController thetaController = new PIDController(Constants.AutoConstants.kThetaPIDConstants.kP,
            Constants.AutoConstants.kThetaPIDConstants.kI, Constants.AutoConstants.kThetaPIDConstants.kD);

    /**
     * Point towards, and move towards, a detected
     * {@link org.photonvision.targeting.PhotonTrackedTarget#PhotonTrackedTarget()
     * PhotonTrackedTarget}
     * 
     * @param driveSubsystem  subsystem used for driving
     * @param cameraSubsystem subsystem containing the camera
     * @param xSpeed
     * @param ySpeed
     * @param fieldRelative
     * @param rateLimit
     * @param slow
     * @param throttle
     */
    public DriveAndOrientToTarget(DriveSubsystem driveSubsystem, IntakeCameraSubsystem cameraSubsystem,
            DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot,
            BooleanSupplier fieldRelative,
            BooleanSupplier rateLimit, BooleanSupplier slow, DoubleSupplier throttle) {
        m_driveSubsystem = driveSubsystem;
        m_cameraSubsytem = cameraSubsystem;

        if (slow.getAsBoolean()) {
            m_xSpeed = () -> xSpeed.getAsDouble() * Constants.DriveConstants.kSlowModifier;
            m_ySpeed = () -> ySpeed.getAsDouble() * Constants.DriveConstants.kSlowModifier;
            m_rot = () -> rot.getAsDouble() * Constants.DriveConstants.kSlowModifier;
        } else {
            m_xSpeed = () -> xSpeed.getAsDouble() * MathUtil.clamp(throttle.getAsDouble(), 0.1, 1.0);
            m_ySpeed = () -> ySpeed.getAsDouble() * MathUtil.clamp(throttle.getAsDouble(), 0.1, 1.0);
            m_rot = () -> rot.getAsDouble() * MathUtil.clamp(throttle.getAsDouble(), 0.1, 1.0);
        }

        m_fieldRelative = fieldRelative;
        m_rateLimit = rateLimit;
        addRequirements(m_cameraSubsytem, m_driveSubsystem);
    }

    @Override
    public void execute() {
        if (counter % 5 == 0) {
            target = m_cameraSubsytem.getTarget();
            if (target != null) {
                // set theta based on yaw
                goalYaw = Math.toRadians(target.getYaw() + Constants.Sensors.gyro.getAngle());
                // System.out.println(goalYaw);
            }
        }
        // check if target is present
        if (target == null) {
            thetaOutput = m_rot.getAsDouble();
            System.out.println("ruh roh");
        } else {
            thetaOutput = thetaController.calculate(Math.toRadians(Constants.Sensors.gyro.getAngle()), goalYaw);
            System.out.println("hey it works");
        }

        m_driveSubsystem.drive(m_xSpeed.getAsDouble(), m_ySpeed.getAsDouble(), thetaOutput,
                m_fieldRelative.getAsBoolean(),
                m_rateLimit.getAsBoolean());
        counter++;

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
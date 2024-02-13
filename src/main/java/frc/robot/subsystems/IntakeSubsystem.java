package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.CustomUtils;

public class IntakeSubsystem extends SubsystemBase {
    private double speed;
    private double angle;

    /**
     * Subsystem for controlling the intake
     */
    public IntakeSubsystem() {
        // TODO: determine whether to invert this or not
        Constants.IntakeConstants.kIntakeAngleMotor.setInverted(Constants.IntakeConstants.kIntakeAngleInverted);
        Constants.IntakeConstants.kIntakeMotor.setInverted(Constants.IntakeConstants.kIntakeInverted);

        //set idle mode
        Constants.IntakeConstants.kIntakeAngleMotor.setIdleMode(Constants.IntakeConstants.kIntakeAngleIdleMode);
        Constants.IntakeConstants.kIntakeMotor.setIdleMode(Constants.IntakeConstants.kIntakeIdleMode);

        // setup PID
        CustomUtils.setSparkPID(Constants.IntakeConstants.kintakeAngleController,
                Constants.IntakeConstants.kPIDConstants);
        Constants.IntakeConstants.kintakeAngleController
                .setFeedbackDevice(Constants.IntakeConstants.kintakeAngleEncoder);

        // Converts to degrees
        Constants.IntakeConstants.kintakeAngleEncoder
                .setPositionConversionFactor(Constants.IntakeConstants.kintakeAngleConversionFactor);
    }

    public void periodic() {
        log();
        Constants.IntakeConstants.kintakeAngleController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Set the speed of the intake
     * 
     * @param speed (-1 to 1)
     */
    public void setSpeed(double speed) {
        this.speed = speed;
        Constants.IntakeConstants.kIntakeMotor.set(speed);
    }

    /**
     * Set the angle of the intake intakeAngle
     * 
     * @param angle
     */
    public void setintakeAnglePosition(double angle) {
        angle = MathUtil.clamp(angle, Constants.IntakeConstants.kintakeAngleLimits[0],
                Constants.IntakeConstants.kintakeAngleLimits[1]);
        this.angle = angle;
    }

    /**
     * 
     * @return whether the intakeAngle is at the set position
     */
    public boolean atPosition() {
        return Math.abs(
                Constants.IntakeConstants.kintakeAngleEncoder.getPosition()
                        - angle) < Constants.IntakeConstants.kTolerance;
    }

    public void log() {
        SmartDashboard.putNumber("Intake Speed", speed);
    }
}

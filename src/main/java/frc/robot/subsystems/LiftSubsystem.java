package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.CustomUtils;

public class LiftSubsystem extends SubsystemBase {
    public double position;

    public LiftSubsystem() {
        // TODO
        Constants.LiftConstants.kLiftMotor.setInverted(Constants.LiftConstants.kInverted);

        //idle mode
        Constants.LiftConstants.kLiftMotor.setIdleMode(Constants.LiftConstants.kIdleMode);

        // setup PID
        CustomUtils.setSparkPID(Constants.LiftConstants.kLiftController, Constants.LiftConstants.kLiftPIDConstants);
        Constants.LiftConstants.kLiftController.setFeedbackDevice(Constants.LiftConstants.kLiftEncoder);

        Constants.LiftConstants.kLiftEncoder.setPositionConversionFactor(Constants.LiftConstants.kLiftConversionFactor);
    }

    public void periodic() {
        log();
        Constants.LiftConstants.kLiftController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Set the position of the lift
     * 
     * @param position (inches)
     */
    public void setPosition(double position) {
        if ((Constants.ShooterConstants.kWristEncoder.getPosition() > Constants.LiftConstants.kWristTolerance &&
                Constants.IntakeConstants.kintakeAngleEncoder
                        .getPosition() < Constants.LiftConstants.kIntakeTolerance)
                || this.position > 3/* TODO protect from the wrist hitting stuff*/) {

            position = MathUtil.clamp(position, Constants.LiftConstants.kLiftLimits[0],
                    Constants.LiftConstants.kLiftLimits[1]);
            this.position = position;

        }

    }

    /**
     * 
     * @return the position of the lift
     */
    public double getPosition() {
        return position;
    }

    /**
     * 
     * @return whether the lift is at the commanded position
     */
    public boolean atPosition() {
        return Math.abs(
                Constants.LiftConstants.kLiftEncoder.getPosition() - position) < Constants.LiftConstants.kTolerance;
    }

    public void log() {
        SmartDashboard.putNumber("Lift Position: ", position);
    }
}

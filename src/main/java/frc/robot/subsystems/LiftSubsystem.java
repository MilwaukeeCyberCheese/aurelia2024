package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.CustomUtils;

public class LiftSubsystem extends SubsystemBase {
    public double position;

    public LiftSubsystem() {
        Constants.LiftConstants.kLiftMotor.restoreFactoryDefaults();

        Constants.LiftConstants.kLiftMotor.setInverted(Constants.LiftConstants.kInverted);

        // idle mode
        Constants.LiftConstants.kLiftMotor.setIdleMode(Constants.LiftConstants.kIdleMode);

        // setup PID
        CustomUtils.setSparkPID(Constants.LiftConstants.kLiftController, Constants.LiftConstants.kLiftPIDConstants);
        Constants.LiftConstants.kLiftController.setFeedbackDevice(Constants.LiftConstants.kLiftMotor.getEncoder());

        Constants.LiftConstants.kLiftEncoder.setPositionConversionFactor(Constants.LiftConstants.kLiftConversionFactor);
        Constants.LiftConstants.kLiftMotor.getEncoder()
                .setPositionConversionFactor(Constants.LiftConstants.kLiftConversionFactorOnboard);

        Constants.LiftConstants.kLiftMotor.getEncoder()
                .setPosition((Constants.LiftConstants.kLiftEncoder.getPosition() < 2.5)
                        ? Constants.LiftConstants.kLiftEncoder.getPosition()
                        : Constants.LiftConstants.kLiftConversionFactor * -1.0
                                + Constants.LiftConstants.kLiftEncoder.getPosition());
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

        position = MathUtil.clamp(position, Constants.LiftConstants.kLiftLimits[0],
                Constants.LiftConstants.kLiftLimits[1]);

        if (getPosition() > Constants.LiftConstants.kClearOfObstructions) {
            this.position = position;
        } else if (RobotContainer.m_intakeSubsystem.getPosition() < 160
                && RobotContainer.m_shooterSubsystem.getPosition() == 90) {
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
     * zero the lift
     * 
     */
    public void zero() {
        Constants.LiftConstants.kLiftMotor.getEncoder().setPosition(0);
        double newOffset = Constants.LiftConstants.kLiftEncoder.getZeroOffset()
                + Constants.LiftConstants.kLiftEncoder.getPosition();
        position = 0.0;
        Constants.LiftConstants.kLiftEncoder.setZeroOffset(newOffset);

    }

    /**
     * 
     * @return whether the lift is at the commanded position
     */
    public boolean atPosition() {
        return Math.abs(
                Constants.LiftConstants.kLiftMotor.getEncoder().getPosition()
                        - position) < Constants.LiftConstants.kLiftTolerance;
    }

    public void log() {
        SmartDashboard.putNumber("Lift Position: ", Constants.LiftConstants.kLiftMotor.getEncoder().getPosition());
    }
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.CustomUtils;
import frc.robot.utils.DashboardUpdater;
import frc.robot.utils.LivePIDTuner;

public class LiftSubsystem extends SubsystemBase {
    public double position;
    private final LivePIDTuner tuner;
    private final DashboardUpdater<Double> positionUpdater;

    public LiftSubsystem() {
        Constants.LiftConstants.kLiftMotor.restoreFactoryDefaults();
        // TODO: determine whether to invert this or not
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

        tuner = new LivePIDTuner("Lift Tuner", Constants.LiftConstants.kLiftController,
                Constants.LiftConstants.kLiftPIDConstants);
        positionUpdater = new DashboardUpdater<Double>("Lift Position Updater", 0.0);
    }

    public void periodic() {
        log();
        tuner.update();
        positionUpdater.update();

        Constants.LiftConstants.kLiftController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Set the position of the lift
     * 
     * @param position (inches)
     */
    public void setPosition(double position) {
        if ((RobotContainer.m_shooterSubsystem.getPosition() > Constants.SafetyLimits.kWristLowerLift &&
                RobotContainer.m_intakeSubsystem.getPosition() < Constants.SafetyLimits.kIntakeUpperLift)
                || (this.position > Constants.LiftConstants.kClearOfObstructions
                        && Constants.LiftConstants.kClearOfObstructions > 3)/*
                                                                             * TODO protect from the wrist hitting stuff
                                                                             */) {
        }

        position = MathUtil.clamp(position, Constants.LiftConstants.kLiftLimits[0],
                Constants.LiftConstants.kLiftLimits[1]);
        this.position = position;

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
                Constants.LiftConstants.kLiftEncoder.getPosition() - position) < Constants.LiftConstants.kTolerance;
    }

    public void log() {
        SmartDashboard.putNumber("Lift Position: ", Constants.LiftConstants.kLiftMotor.getEncoder().getPosition());
    }
}

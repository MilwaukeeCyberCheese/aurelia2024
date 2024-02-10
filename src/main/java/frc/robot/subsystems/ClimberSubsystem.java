package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.CustomUtils;

public class ClimberSubsystem extends SubsystemBase {
    private double leftPosition;
    private double rightPosition;

    public ClimberSubsystem() {
        // inverted
        Constants.ClimberConstants.kLeftMotor.setInverted(Constants.ClimberConstants.kLeftInverted);
        Constants.ClimberConstants.kRightMotor.setInverted(Constants.ClimberConstants.kRightInverted);

        //idle mode
        Constants.ClimberConstants.kLeftMotor.setIdleMode(Constants.ClimberConstants.kLeftIdleMode);
        Constants.ClimberConstants.kRightMotor.setIdleMode(Constants.ClimberConstants.kRightIdleMode);

        // setup PID
        CustomUtils.setSparkPID(Constants.ClimberConstants.kLeftController, Constants.ClimberConstants.kPIDConstants);
        Constants.ClimberConstants.kLeftController
                .setFeedbackDevice(Constants.ClimberConstants.kLeftEncoder);

        CustomUtils.setSparkPID(Constants.ClimberConstants.kRightController, Constants.ClimberConstants.kPIDConstants);
        Constants.ClimberConstants.kRightController
                .setFeedbackDevice(Constants.ClimberConstants.kRightEncoder);

        Constants.ClimberConstants.kLeftEncoder
                .setPositionConversionFactor(Constants.ClimberConstants.kConversionFactor);
        Constants.ClimberConstants.kRightEncoder
                .setPositionConversionFactor(Constants.ClimberConstants.kConversionFactor);
    }

    public void setPosition(double position) {
        this.leftPosition = position;
        this.rightPosition = position;
    }

    public boolean atPosition() {
        return Math
                .abs(Constants.ClimberConstants.kLeftEncoder.getPosition()
                        - leftPosition) < Constants.ClimberConstants.kTolerance
                && Math.abs(Constants.ClimberConstants.kRightEncoder.getPosition()
                        - rightPosition) < Constants.ClimberConstants.kTolerance;
    }

    public void zero() {
        Constants.ClimberConstants.kLeftEncoder.setPosition(0);
        Constants.ClimberConstants.kRightEncoder.setPosition(0);
    }

    public void periodic() {
        log();

        Constants.ClimberConstants.kLeftController.setReference(leftPosition, CANSparkMax.ControlType.kPosition);
        Constants.ClimberConstants.kRightController.setReference(rightPosition, CANSparkMax.ControlType.kPosition);
    }

    public void log() {

    }
}

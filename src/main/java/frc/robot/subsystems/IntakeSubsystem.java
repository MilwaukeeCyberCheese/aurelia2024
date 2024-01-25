package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private double speed;
    private double angle;

    /**
     * Subsystem for controlling the intake
     */
    public IntakeSubsystem() {
        // TODO
        Constants.IntakeConstants.kPivotMotor.setInverted(true);
        Constants.IntakeConstants.kIntakeMotor.setInverted(true);

        // setup PID
        Constants.IntakeConstants.kPivotController.setP(Constants.IntakeConstants.kPivotPIDConstants.kP);
        Constants.IntakeConstants.kPivotController.setI(Constants.IntakeConstants.kPivotPIDConstants.kI);
        Constants.IntakeConstants.kPivotController.setD(Constants.IntakeConstants.kPivotPIDConstants.kD);
        Constants.IntakeConstants.kPivotController.setFeedbackDevice(Constants.IntakeConstants.kPivotEncoder);

        // Converts to degrees
        Constants.IntakeConstants.kPivotEncoder
                .setPositionConversionFactor(Constants.IntakeConstants.kPivotConversionFactor);
    }

    public void periodic() {
        log();
        Constants.IntakeConstants.kPivotController.setReference(angle, CANSparkMax.ControlType.kPosition);
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
     * Set the angle of the intake pivot
     * 
     * @param angle
     */
    public void setPivotPosition(double angle) {
        angle = MathUtil.clamp(angle, Constants.IntakeConstants.kPivotLimits[0],
                Constants.IntakeConstants.kPivotLimits[1]);
        this.angle = angle;
    }

    /**
     * 
     * @return whether the pivot is at the set position
     */
    public boolean atPosition() {
        return Math.abs(
                Constants.IntakeConstants.kPivotEncoder.getPosition() - angle) < Constants.IntakeConstants.kTolerance;
    }

    public void log() {
        SmartDashboard.putNumber("Intake Speed", speed);
    }
}

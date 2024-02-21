package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.CustomUtils;
import frc.robot.utils.DashboardUpdater;
import frc.robot.utils.LivePIDTuner;

public class IntakeSubsystem extends SubsystemBase {
    private double speed;
    private double angle;
    private LivePIDTuner tuner;
    private DashboardUpdater<Double> position;

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
                .setPositionConversionFactor(Constants.IntakeConstants.kIntakeAngleConversionFactor);
        tuner = new LivePIDTuner("Intake Tuner", Constants.IntakeConstants.kintakeAngleController, Constants.IntakeConstants.kPIDConstants);
        position = new DashboardUpdater<>("Intake Position", 0.0);
        
    }

    public void periodic() {
        log();
        // tuner.update();

        Constants.IntakeConstants.kintakeAngleController.setReference(20, CANSparkMax.ControlType.kPosition);
        Constants.IntakeConstants.kIntakeMotor.set(speed);
    }

    /**
     * Set the speed of the intake
     * 
     * @param speed (-1 to 1)
     */
    public void setSpeed(double speed) {
        this.speed = MathUtil.clamp(speed, -1.0, 1.0);
        System.out.println(speed);
        
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
        SmartDashboard.putNumber("Intake angle speed", Constants.IntakeConstants.kIntakeAngleMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Intake Angle", Constants.IntakeConstants.kintakeAngleEncoder.getPosition());
    }
}

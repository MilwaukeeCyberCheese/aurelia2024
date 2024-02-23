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
        // private LivePIDTuner tuner;
        // private DashboardUpdater<Double> position;

        /**
         * Subsystem for controlling the intake
         */
        public IntakeSubsystem() {
                // reset sparkmax
                Constants.IntakeConstants.kIntakeAngleMotor.restoreFactoryDefaults();
                // TODO: determine whether to invert the intake motor
                Constants.IntakeConstants.kIntakeAngleMotor.setInverted(Constants.IntakeConstants.kIntakeAngleInverted);
                Constants.IntakeConstants.kIntakeMotor.setInverted(Constants.IntakeConstants.kIntakeInverted);

                // set idle mode
                Constants.IntakeConstants.kIntakeAngleMotor.setIdleMode(Constants.IntakeConstants.kIntakeAngleIdleMode);
                Constants.IntakeConstants.kIntakeMotor.setIdleMode(Constants.IntakeConstants.kIntakeIdleMode);

                // setup PID
                CustomUtils.setSparkPID(Constants.IntakeConstants.kIntakeAngleController,
                                Constants.IntakeConstants.kPIDConstants);
                Constants.IntakeConstants.kIntakeAngleController
                                .setFeedbackDevice(Constants.IntakeConstants.kIntakeAngleEncoder);

                // limit PID output
                Constants.IntakeConstants.kIntakeAngleController.setOutputRange(
                                Constants.IntakeConstants.kIntakeAngleMaxOuput * -1.0,
                                Constants.IntakeConstants.kIntakeAngleMaxOuput);

                // Converts to degrees
                Constants.IntakeConstants.kIntakeAngleEncoder
                                .setPositionConversionFactor(Constants.IntakeConstants.kIntakeAngleConversionFactor);
                Constants.IntakeConstants.kIntakeAngleEncoder.setInverted(true);

                // Different stuff for tuning
                // tuner = new LivePIDTuner("Intake Tuner", Constants.IntakeConstants.kIntakeAngleController,
                //                 Constants.IntakeConstants.kPIDConstants);
                // position = new DashboardUpdater<Double>("Intake Position", 9.0);

        }

        public void periodic() {
                log();
                // tuner.update();
                // position.update();
                Constants.IntakeConstants.kIntakeAngleController.setReference(angle,
                                CANSparkMax.ControlType.kPosition);
                Constants.IntakeConstants.kIntakeMotor.set(speed);
        }

        /**
         * Set the speed of the intake
         * 
         * @param speed (-1 to 1)
         */
        public void setSpeed(double speed) {
                this.speed = MathUtil.clamp(speed, -1.0, 1.0);

        }

        /**
         * Set the angle of the intake intakeAngle
         * 
         * @param angle
         */
        public void setintakeAnglePosition(double angle) {
                angle = MathUtil.clamp(angle, Constants.IntakeConstants.kIntakeAngleLimits[0],
                                Constants.IntakeConstants.kIntakeAngleLimits[1]);
                this.angle = angle;
        }

        /**
         * 
         * @return whether the intakeAngle is at the set position
         */
        public boolean atPosition() {
                return Math.abs(
                                Constants.IntakeConstants.kIntakeAngleEncoder.getPosition()
                                                - angle) < Constants.IntakeConstants.kTolerance;
        }

        public void log() {
                SmartDashboard.putNumber("Intake Angle", Constants.IntakeConstants.kIntakeAngleEncoder.getPosition());
                // SmartDashboard.putNumber("Intake P: ", Constants.IntakeConstants.kIntakeAngleController.getP());
                // SmartDashboard.putNumber("Intake I: ", Constants.IntakeConstants.kIntakeAngleController.getI());
                // SmartDashboard.putNumber("Intake D: ", Constants.IntakeConstants.kIntakeAngleController.getD());
                // SmartDashboard.putNumber("Intake FF: ", Constants.IntakeConstants.kIntakeAngleController.getFF());
        }
}

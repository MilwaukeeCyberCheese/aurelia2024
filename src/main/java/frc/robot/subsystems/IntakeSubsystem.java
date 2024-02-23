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
        private double position;
        // private LivePIDTuner tuner;
        // private DashboardUpdater<Double> position;

        /**
         * Subsystem for controlling the intake
         */
        public IntakeSubsystem() {
                // reset sparkmax
                Constants.IntakeConstants.kIntakePivotMotor.restoreFactoryDefaults();
                // TODO: determine whether to invert the intake motor
                Constants.IntakeConstants.kIntakePivotMotor.setInverted(Constants.IntakeConstants.kIntakePivotInverted);
                Constants.IntakeConstants.kIntakeMotor.setInverted(Constants.IntakeConstants.kIntakeInverted);

                // set idle mode
                Constants.IntakeConstants.kIntakePivotMotor.setIdleMode(Constants.IntakeConstants.kIntakePivotIdleMode);
                Constants.IntakeConstants.kIntakeMotor.setIdleMode(Constants.IntakeConstants.kIntakeIdleMode);

                // setup PID
                CustomUtils.setSparkPID(Constants.IntakeConstants.kIntakePositionController,
                                Constants.IntakeConstants.kPIDConstants);
                Constants.IntakeConstants.kIntakePositionController
                                .setFeedbackDevice(Constants.IntakeConstants.kIntakePositionEncoder);

                // limit PID output
                Constants.IntakeConstants.kIntakePositionController.setOutputRange(
                                Constants.IntakeConstants.kIntakePivotMaxOuput * -1.0,
                                Constants.IntakeConstants.kIntakePivotMaxOuput);

                // Converts to degrees
                Constants.IntakeConstants.kIntakePositionEncoder
                                .setPositionConversionFactor(Constants.IntakeConstants.kIntakePositionConversionFactor);
                Constants.IntakeConstants.kIntakePositionEncoder
                                .setInverted(Constants.IntakeConstants.kIntakePositionEncoderInverted);

                // Different stuff for tuning
                // tuner = new LivePIDTuner("Intake Tuner",
                // Constants.IntakeConstants.kIntakeAngleController,
                // Constants.IntakeConstants.kPIDConstants);
                // position = new DashboardUpdater<Double>("Intake Position", 9.0);

        }

        public void periodic() {
                log();
                // tuner.update();
                // position.update();
                Constants.IntakeConstants.kIntakePositionController.setReference(position,
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
         * Set the position of the intake
         * 
         * @param position
         */
        public void setPosition(double position) {
                position = MathUtil.clamp(position, Constants.IntakeConstants.kIntakePositionLimits[0],
                                Constants.IntakeConstants.kIntakePositionLimits[1]);
                if ((this.position < Constants.SafetyLimits.kIntakeUpperLift
                                && position < Constants.SafetyLimits.kIntakeUpperLift) // TODO: add lift safety checks
                ) {
                        this.position = position;
                }
        }

        /**
         * 
         * @return whether the intake is at the set position
         */
        public boolean atPosition() {
                return Math.abs(
                                Constants.IntakeConstants.kIntakePositionEncoder.getPosition()
                                                - position) < Constants.IntakeConstants.kTolerance;
        }

        /**
         * @return position of the intake
         */
        public double getPosition() {
                return position;
        }

        public void log() {
                SmartDashboard.putNumber("Intake Angle",
                                Constants.IntakeConstants.kIntakePositionEncoder.getPosition());
                // SmartDashboard.putNumber("Intake P: ",
                // Constants.IntakeConstants.kIntakeAngleController.getP());
                // SmartDashboard.putNumber("Intake I: ",
                // Constants.IntakeConstants.kIntakeAngleController.getI());
                // SmartDashboard.putNumber("Intake D: ",
                // Constants.IntakeConstants.kIntakeAngleController.getD());
                // SmartDashboard.putNumber("Intake FF: ",
                // Constants.IntakeConstants.kIntakeAngleController.getFF());
        }
}

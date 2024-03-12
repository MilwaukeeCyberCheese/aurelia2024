package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.CustomUtils;
import frc.robot.utils.DashboardUpdater;

public class IntakeSubsystem extends SubsystemBase {
        private double speed = 0.0;
        private double position = Constants.IntakeConstants.kIntakeLoadPosition;
        private DashboardUpdater<Double> positionUpdater;
        private DashboardUpdater<Double> speedUpdater;

        /**
         * Subsystem for controlling the intake
         */
        public IntakeSubsystem() {
                // reset sparkmax
                Constants.IntakeConstants.kIntakePivotMotor.restoreFactoryDefaults();
                // TODO: determine whether to invert the intake motor
                Constants.IntakeConstants.kIntakePivotMotor.setInverted(Constants.IntakeConstants.kIntakePivotInverted);
                Constants.IntakeConstants.kIntakeMotor.setInverted(Constants.IntakeConstants.kIntakeInverted);
                Constants.IntakeConstants.kIntakeMotor.setControlFramePeriodMs(0);
                // set idle mode
                Constants.IntakeConstants.kIntakePivotMotor.setIdleMode(Constants.IntakeConstants.kIntakePivotIdleMode);
                Constants.IntakeConstants.kIntakeMotor.setIdleMode(Constants.IntakeConstants.kIntakeIdleMode);

                //current limits
                Constants.IntakeConstants.kIntakeMotor.setSmartCurrentLimit(Constants.IntakeConstants.kIntakeCurrentLimit);

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

                positionUpdater = new DashboardUpdater<Double>("Intake Position Updater", 15.0);
                speedUpdater = new DashboardUpdater<Double>("Intake Speed Updater", 0.0);

        }

        public void periodic() {
                log();
                positionUpdater.update();
                speedUpdater.update();
                if (RobotContainer.m_shooterSubsystem.getPosition() > 115) {
                        Constants.IntakeConstants.kIntakePositionController.setReference(position,
                                        CANSparkMax.ControlType.kPosition);
                        
                }
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
                // if ((this.position < Constants.SafetyLimits.kIntakeUpperLift
                // && position < Constants.SafetyLimits.kIntakeUpperLift) // TODO: add lift
                // safety checks
                // ) {
                this.position = position;
                // }
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
                SmartDashboard.putNumber("Pivot Applied Output",
                                Constants.IntakeConstants.kIntakePivotMotor.getAppliedOutput());
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

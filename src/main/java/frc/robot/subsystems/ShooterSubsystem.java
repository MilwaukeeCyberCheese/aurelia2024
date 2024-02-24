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

public class ShooterSubsystem extends SubsystemBase {
        private double leftRPM;
        private double rightRPM;
        private double position;
        // private LivePIDTuner leftTuner;
        // private LivePIDTuner rightTuner;
        // private LivePIDTuner wristTuner;
        private DashboardUpdater<Double> positionUpdater;
        private DashboardUpdater<Double> rpmUpdater;

        public ShooterSubsystem() {
                Constants.ShooterConstants.kWristMotor.restoreFactoryDefaults();

                // inverted
                Constants.ShooterConstants.kLeftShooterMotor.setInverted(Constants.ShooterConstants.kLeftInverted);
                Constants.ShooterConstants.kRightShooterMotor.setInverted(Constants.ShooterConstants.kRightInverted);
                Constants.ShooterConstants.kWristMotor.setInverted(Constants.ShooterConstants.kWristInverted);
                Constants.ShooterConstants.kWristEncoder.setInverted(Constants.ShooterConstants.kWristEncoderInverted);

                // set idle mode
                Constants.ShooterConstants.kLeftShooterMotor.setIdleMode(Constants.ShooterConstants.kShooterIdleMode);
                Constants.ShooterConstants.kRightShooterMotor.setIdleMode(Constants.ShooterConstants.kShooterIdleMode);
                Constants.ShooterConstants.kWristMotor.setIdleMode(Constants.ShooterConstants.kWristIdleMode);

                // setup PID
                CustomUtils.setSparkPID(Constants.ShooterConstants.kLeftShooterController,
                                Constants.ShooterConstants.kShooterPIDConstants);
                Constants.ShooterConstants.kLeftShooterController
                                .setFeedbackDevice(Constants.ShooterConstants.kLeftShooterEncoder);

                CustomUtils.setSparkPID(Constants.ShooterConstants.kRightShooterController,
                                Constants.ShooterConstants.kShooterPIDConstants);
                Constants.ShooterConstants.kRightShooterController
                                .setFeedbackDevice(Constants.ShooterConstants.kRightShooterEncoder);

                CustomUtils.setSparkPID(Constants.ShooterConstants.kWristController,
                                Constants.ShooterConstants.kWristPIDConstants);
                Constants.ShooterConstants.kWristController
                                .setFeedbackDevice(Constants.ShooterConstants.kWristEncoder);

                Constants.ShooterConstants.kLeftShooterEncoder
                                .setVelocityConversionFactor(Constants.ShooterConstants.kShooterConversionFactor);
                Constants.ShooterConstants.kRightShooterEncoder
                                .setVelocityConversionFactor(Constants.ShooterConstants.kShooterConversionFactor);
                Constants.ShooterConstants.kWristEncoder
                                .setPositionConversionFactor(Constants.ShooterConstants.kWristConversionFactor);

                Constants.ShooterConstants.kWristController.setOutputRange(
                                Constants.ShooterConstants.kWristMaxOutput * -1.0,
                                Constants.ShooterConstants.kWristMaxOutput);

                // live PID tuner
                // leftTuner = new LivePIDTuner("Left Shooter",
                //                 Constants.ShooterConstants.kLeftShooterController,
                //                 Constants.ShooterConstants.kShooterPIDConstants);
                // rightTuner = new LivePIDTuner("Right Shooter",
                //                 Constants.ShooterConstants.kRightShooterController,
                //                 Constants.ShooterConstants.kShooterPIDConstants);
                // wristTuner = new LivePIDTuner("Wrist Tuner", Constants.ShooterConstants.kWristController,
                //                 Constants.ShooterConstants.kWristPIDConstants);
                positionUpdater = new DashboardUpdater<Double>("Wrist Position", 190.0);
                rpmUpdater = new DashboardUpdater<Double>("RPM", 0.0);
        }

        /**
         * Set RPM of both sides of the shooter
         * 
         * @param rpm
         */
        public void setRPM(double rpm) {
                setRPMs(rpm, rpm);
        }

        /**
         * Set RPMs of sides of the shooter individually
         * 
         * @param leftRPM
         * @param rightRPM
         */
        public void setRPMs(double leftRPM, double rightRPM) {
                leftRPM = MathUtil.clamp(leftRPM, 0, Constants.ShooterConstants.kMaxRPM);
                rightRPM = MathUtil.clamp(rightRPM, 0, Constants.ShooterConstants.kMaxRPM);
                this.leftRPM = leftRPM;
                this.rightRPM = rightRPM;
        }

        /**
         * 
         * @return whether the shooters are at the commanded RPM
         */
        public boolean atRPM() {
                return Math
                                .abs(Constants.ShooterConstants.kLeftShooterEncoder.getVelocity()
                                                - leftRPM) < Constants.ShooterConstants.kShooterTolerance
                                &&
                                Math.abs(Constants.ShooterConstants.kRightShooterEncoder.getVelocity()
                                                - rightRPM) < Constants.ShooterConstants.kShooterTolerance;
        }

        /**
         * Set position for the wrist to go to
         * 
         * @param position (degrees)
         */
        public void setPosition(double position) {
                MathUtil.clamp(position, Constants.ShooterConstants.kWristLimits[0],
                                Constants.ShooterConstants.kWristLimits[1]);
                                //TODO: make these constants and add more limits if needed
                if (RobotContainer.m_liftSubsystem.getPosition() > 3 || (position > 60 && this.position > 60)) {

                        this.position = position;
                }
        }

        /**
         * 
         * @return whether the wrist is at the commanded position
         */
        public boolean atPosition() {
                return Math.abs(Constants.ShooterConstants.kWristEncoder.getPosition()
                                - position) < Constants.ShooterConstants.kWristTolerance;
        }

        /**
         * @return the position
         */
        public double getPosition() {
                return position;
        }

        public void periodic() {
                log();

                // leftTuner.update();
                // rightTuner.update();
                // wristTuner.update();
                positionUpdater.update();
                rpmUpdater.update();

                Constants.ShooterConstants.kLeftShooterController.setReference(leftRPM,
                                CANSparkMax.ControlType.kVelocity);
                Constants.ShooterConstants.kRightShooterController.setReference(rightRPM,
                                CANSparkMax.ControlType.kVelocity);
                Constants.ShooterConstants.kWristController.setReference(positionUpdater.get(),
                CANSparkMax.ControlType.kPosition);
        }

        public void log() {
                // SmartDashboard.putNumber("Left RPM", leftRPM);
                // SmartDashboard.putNumber("Right RPM", rightRPM);
                // SmartDashboard.putNumber("Left RPM Actual",
                // Constants.ShooterConstants.kLeftShooterEncoder.getVelocity());
                // SmartDashboard.putNumber("Right RPM Actual",
                // Constants.ShooterConstants.kRightShooterEncoder.getVelocity());
                SmartDashboard.putNumber("Wrist Position Actual",
                                Constants.ShooterConstants.kWristEncoder.getPosition());
                SmartDashboard.putNumber("Wrist Speed", Constants.ShooterConstants.kWristMotor.getEncoder().getVelocity());
        }
}

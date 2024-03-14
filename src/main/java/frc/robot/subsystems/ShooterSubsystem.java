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
        private double upperRPM = 0;
        private double lowerRPM = 0;
        private double position = Constants.ShooterConstants.kIntakeSafeAngle;
        // private LivePIDTuner shooterTuner;
        private LivePIDTuner wristTuner;
        private DashboardUpdater<Double> positionUpdater;
        // private DashboardUpdater<Double> rpmUpdater;

        public ShooterSubsystem() {
                Constants.ShooterConstants.kWristMotor.restoreFactoryDefaults();

                // inverted
                Constants.ShooterConstants.kUpperShooterMotor
                                .setInverted(Constants.ShooterConstants.kUpperShooterInverted);
                Constants.ShooterConstants.kLowerShooterMotor
                                .setInverted(Constants.ShooterConstants.kLowerShooterInverted);
                Constants.ShooterConstants.kWristMotor.setInverted(Constants.ShooterConstants.kWristInverted);
                Constants.ShooterConstants.kWristEncoder.setInverted(Constants.ShooterConstants.kWristEncoderInverted);

                // set idle mode
                Constants.ShooterConstants.kUpperShooterMotor.setIdleMode(Constants.ShooterConstants.kShooterIdleMode);
                Constants.ShooterConstants.kLowerShooterMotor.setIdleMode(Constants.ShooterConstants.kShooterIdleMode);
                Constants.ShooterConstants.kWristMotor.setIdleMode(Constants.ShooterConstants.kWristIdleMode);

                // setup current limits
                Constants.ShooterConstants.kUpperShooterMotor
                                .setSmartCurrentLimit(Constants.ShooterConstants.kUpperCurrentLimit);
                Constants.ShooterConstants.kLowerShooterMotor
                                .setSmartCurrentLimit(Constants.ShooterConstants.kLowerCurrentLimit);

                CustomUtils.setSparkPID(Constants.ShooterConstants.kUpperShooterController,
                                Constants.ShooterConstants.kUpperShooterPIDConstants);
                Constants.ShooterConstants.kUpperShooterController
                                .setFeedbackDevice(Constants.ShooterConstants.kUpperShooterEncoder);

                CustomUtils.setSparkPID(Constants.ShooterConstants.kLowerShooterController,
                                Constants.ShooterConstants.kLowerShooterPIDConstants);
                Constants.ShooterConstants.kLowerShooterController
                                .setFeedbackDevice(Constants.ShooterConstants.kLowerShooterEncoder);

                CustomUtils.setSparkPID(Constants.ShooterConstants.kWristController,
                                Constants.ShooterConstants.kWristPIDConstants);
                Constants.ShooterConstants.kWristController
                                .setFeedbackDevice(Constants.ShooterConstants.kWristEncoder);

                Constants.ShooterConstants.kUpperShooterEncoder
                                .setVelocityConversionFactor(Constants.ShooterConstants.kShooterConversionFactor);
                Constants.ShooterConstants.kLowerShooterEncoder
                                .setVelocityConversionFactor(Constants.ShooterConstants.kShooterConversionFactor);
                Constants.ShooterConstants.kWristEncoder
                                .setPositionConversionFactor(Constants.ShooterConstants.kWristConversionFactor);

                Constants.ShooterConstants.kWristController.setOutputRange(
                                Constants.ShooterConstants.kWristMaxOutput * -1.0,
                                Constants.ShooterConstants.kWristMaxOutput);

                // live PID tuner
                // shooterTuner = new LivePIDTuner("Shooter",
                // Constants.ShooterConstants.kShooterController,
                // Constants.ShooterConstants.kShooterPIDConstants);
                wristTuner = new LivePIDTuner("Wrist Tuner",
                                Constants.ShooterConstants.kWristController,
                                Constants.ShooterConstants.kWristPIDConstants);
                positionUpdater = new DashboardUpdater<Double>("Wrist Position Updater", 90.0);
                // rpmUpdater = new DashboardUpdater<Double>("RPM Updater", 0.0);
        }

        /**
         * Set RPM of both sides of the shooter
         * 
         * @param rpm
         */
        public void setRPM(double rpm) {
                rpm = MathUtil.clamp(rpm, -Constants.ShooterConstants.kMaxRPM, Constants.ShooterConstants.kMaxRPM);
                setUpperRPM(rpm);
                setLowerRPM(rpm);
        }

        public void setUpperRPM(double rpm) {
                this.upperRPM = rpm;
        }

        public void setLowerRPM(double rpm) {
                this.lowerRPM = rpm;
        }

        /**
         * 
         * @return whether the shooters are at the commanded RPM
         */
        public boolean atRPM() {
                return Math.abs(Constants.ShooterConstants.kUpperShooterEncoder.getVelocity()
                                - upperRPM) < Constants.ShooterConstants.kShooterTolerance
                                && Math.abs(Constants.ShooterConstants.kLowerShooterEncoder.getVelocity()
                                                - lowerRPM) < Constants.ShooterConstants.kShooterTolerance;
        }

        /**
         * Set position for the wrist to go to
         * 
         * @param position (degrees)
         */
        public void setPosition(double position) {
                MathUtil.clamp(position, Constants.ShooterConstants.kWristLimits[0],
                                Constants.ShooterConstants.kWristLimits[1]);
                                
                if (RobotContainer.m_liftSubsystem.getPosition() > Constants.LiftConstants.kClearOfObstructions) {
                        this.position = position;
                } else if (RobotContainer.m_intakeSubsystem.getPosition() <= 160 && position > 0 && position < 0) {
                        this.position = position;// TODO: determine limits while lift is down and intake stowed
                } else if (position > 0 && position < 0) {
                        this.position = position;// TODO: determine limits while lift is down and intake lift
                }

        }

        /**
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

                // shooterTuner.update();
                wristTuner.update();
                positionUpdater.update();
                // rpmUpdater.update();

                Constants.ShooterConstants.kUpperShooterController.setReference(upperRPM,
                                CANSparkMax.ControlType.kVelocity);
                Constants.ShooterConstants.kLowerShooterController.setReference(lowerRPM,
                                CANSparkMax.ControlType.kVelocity);
                Constants.ShooterConstants.kWristController.setReference(positionUpdater.get(),
                                CANSparkMax.ControlType.kPosition);
        }

        public void log() {
                SmartDashboard.putBoolean("Wrist at Position", atPosition());
                SmartDashboard.putNumber("Upper Shooter Speed",
                                Constants.ShooterConstants.kUpperShooterMotor.getEncoder().getVelocity());
                SmartDashboard.putNumber("Lower Shooter Speed",
                                Constants.ShooterConstants.kLowerShooterMotor.getEncoder().getVelocity());
                SmartDashboard.putNumber("Wrist Position",
                                Constants.ShooterConstants.kWristEncoder.getPosition());
        }
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import frc.robot.utils.CustomUtils;
import frc.robot.utils.DashboardUpdater;


public class ShooterSubsystem extends SubsystemBase {
        private double RPM = 0;
        private double position = 150;
        // private LivePIDTuner shooterTuner;
        // private LivePIDTuner wristTuner;
        private DashboardUpdater<Double> positionUpdater;
        private DashboardUpdater<Double> rpmUpdater;

        public ShooterSubsystem() {
                Constants.ShooterConstants.kWristMotor.restoreFactoryDefaults();

                // inverted
                Constants.ShooterConstants.kShooterMotor.setInverted(Constants.ShooterConstants.kShooterInverted);
                      Constants.ShooterConstants.kWristMotor.setInverted(Constants.ShooterConstants.kWristInverted);
                Constants.ShooterConstants.kWristEncoder.setInverted(Constants.ShooterConstants.kWristEncoderInverted);

                // set idle mode
                     Constants.ShooterConstants.kShooterMotor.setIdleMode(Constants.ShooterConstants.kShooterIdleMode);
                Constants.ShooterConstants.kWristMotor.setIdleMode(Constants.ShooterConstants.kWristIdleMode);

                // setup current limits
                Constants.ShooterConstants.kShooterMotor.setSmartCurrentLimit(Constants.ShooterConstants.kCurrentLimit);
              

                CustomUtils.setSparkPID(Constants.ShooterConstants.kShooterController,
                                Constants.ShooterConstants.kShooterPIDConstants);
                Constants.ShooterConstants.kShooterController
                                .setFeedbackDevice(Constants.ShooterConstants.kShooterEncoder);

                CustomUtils.setSparkPID(Constants.ShooterConstants.kWristController,
                                Constants.ShooterConstants.kWristPIDConstants);
                Constants.ShooterConstants.kWristController
                                .setFeedbackDevice(Constants.ShooterConstants.kWristEncoder);

                     Constants.ShooterConstants.kShooterEncoder
                                .setVelocityConversionFactor(Constants.ShooterConstants.kShooterConversionFactor);
                Constants.ShooterConstants.kWristEncoder
                                .setPositionConversionFactor(Constants.ShooterConstants.kWristConversionFactor);

                Constants.ShooterConstants.kWristController.setOutputRange(
                                Constants.ShooterConstants.kWristMaxOutput * -1.0,
                                Constants.ShooterConstants.kWristMaxOutput);

                // live PID tuner
                // shooterTuner = new LivePIDTuner("Shooter",
                //                 Constants.ShooterConstants.kShooterController,
                //                 Constants.ShooterConstants.kShooterPIDConstants);
                // wristTuner = new LivePIDTuner("Wrist Tuner", Constants.ShooterConstants.kWristController,
                //                 Constants.ShooterConstants.kWristPIDConstants);
                positionUpdater = new DashboardUpdater<Double>("Wrist Position Updater", 90.0);
                rpmUpdater = new DashboardUpdater<Double>("RPM Updater", 0.0);
        }

        /**
         * Set RPM of both sides of the shooter
         * 
         * @param rpm
         */
        public void setRPM(double rpm) {
                rpm = MathUtil.clamp(rpm, -Constants.ShooterConstants.kMaxRPM, Constants.ShooterConstants.kMaxRPM);
                this.RPM = rpm;
        }

     

       

        /**
         * 
         * @return whether the shooters are at the commanded RPM
         */
        public boolean atRPM() {
                return Math.abs(Constants.ShooterConstants.kShooterEncoder.getVelocity() - RPM) 
                < Constants.ShooterConstants.kShooterTolerance;
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
                // if (RobotContainer.m_liftSubsystem.getPosition() > 3 || (position > 60 && this.position > 60)) {

                        this.position = position;
                // }
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

                // shooterTuner.update();
                // wristTuner.update();
                positionUpdater.update();
                rpmUpdater.update();

                Constants.ShooterConstants.kShooterController.setReference(RPM,
                                CANSparkMax.ControlType.kVelocity);
                // Constants.ShooterConstants.kShooterMotor.set(0.1);
                Constants.ShooterConstants.kWristController.setReference(position,
                CANSparkMax.ControlType.kPosition);
        }

        public void log() {
                SmartDashboard.putBoolean("Shooter at Position", atPosition());
                SmartDashboard.putNumber("Shooter Speed", Constants.ShooterConstants.kShooterMotor.getEncoder().getVelocity());
                SmartDashboard.putNumber("Wrist Position Actual",
                                Constants.ShooterConstants.kWristEncoder.getPosition());
                // SmartDashboard.putNumber("Wrist Speed", Constants.ShooterConstants.kWristMotor.getEncoder().getVelocity());
        }
}

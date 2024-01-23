package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
        private double leftRPM;
        private double rightRPM;
        private double position;

        public ShooterSubsystem() {
                // inverted
                Constants.ShooterConstants.kLeftShooterMotor.setInverted(Constants.ShooterConstants.kLeftInverted);
                Constants.ShooterConstants.kRightShooterMotor.setInverted(Constants.ShooterConstants.kRightInverted);
                Constants.ShooterConstants.kWristMotor.setInverted(Constants.ShooterConstants.kWristInverted);

                // setup PID
                Constants.ShooterConstants.kLeftShooterController
                                .setP(Constants.ShooterConstants.kShooterPIDConstants.kP);
                Constants.ShooterConstants.kLeftShooterController
                                .setI(Constants.ShooterConstants.kShooterPIDConstants.kI);
                Constants.ShooterConstants.kLeftShooterController
                                .setD(Constants.ShooterConstants.kShooterPIDConstants.kD);
                Constants.ShooterConstants.kLeftShooterController
                                .setFeedbackDevice(Constants.ShooterConstants.kLeftShooterEncoder);
                Constants.ShooterConstants.kRightShooterController
                                .setP(Constants.ShooterConstants.kShooterPIDConstants.kP);
                Constants.ShooterConstants.kRightShooterController
                                .setI(Constants.ShooterConstants.kShooterPIDConstants.kI);
                Constants.ShooterConstants.kRightShooterController
                                .setD(Constants.ShooterConstants.kShooterPIDConstants.kD);
                Constants.ShooterConstants.kRightShooterController
                                .setFeedbackDevice(Constants.ShooterConstants.kRightShooterEncoder);
                Constants.ShooterConstants.kWristController.setP(Constants.ShooterConstants.kWristPIDConstants.kP);
                Constants.ShooterConstants.kWristController.setI(Constants.ShooterConstants.kWristPIDConstants.kI);
                Constants.ShooterConstants.kWristController.setD(Constants.ShooterConstants.kWristPIDConstants.kD);
                Constants.ShooterConstants.kWristController
                                .setFeedbackDevice(Constants.ShooterConstants.kWristEncoder);

                Constants.ShooterConstants.kLeftShooterEncoder
                                .setVelocityConversionFactor(Constants.ShooterConstants.kShooterConversionFactor);
                Constants.ShooterConstants.kRightShooterEncoder
                                .setVelocityConversionFactor(Constants.ShooterConstants.kShooterConversionFactor);
                Constants.ShooterConstants.kWristEncoder
                                .setVelocityConversionFactor(Constants.ShooterConstants.kWristConversionFactor);
        }

        public void setRPM(double rpm) {
                setRPMs(rpm, rpm);

        }

        public void setRPMs(double leftRPM, double rightRPM) {
                this.leftRPM = leftRPM;
                this.rightRPM = rightRPM;
        }

        public boolean atRPM() {
                return Math
                                .abs(Constants.ShooterConstants.kLeftShooterEncoder.getVelocity()
                                                - leftRPM) < Constants.ShooterConstants.kShooterTolerance
                                &&
                                Math.abs(Constants.ShooterConstants.kRightShooterEncoder.getVelocity()
                                                - rightRPM) < Constants.ShooterConstants.kShooterTolerance;
        }

        public void setPosition(double position) {
                this.position = position;
        }

        public boolean atPosition() {
                return Math.abs(Constants.ShooterConstants.kWristEncoder.getPosition()
                                - position) < Constants.ShooterConstants.kWristTolerance;
        }

        public void periodic() {
                log();
                
                        Constants.ShooterConstants.kLeftShooterController.setReference(leftRPM,
                                        CANSparkMax.ControlType.kVelocity);
                        Constants.ShooterConstants.kRightShooterController.setReference(rightRPM,
                                        CANSparkMax.ControlType.kVelocity);
                
                Constants.ShooterConstants.kWristController.setReference(position, CANSparkMax.ControlType.kPosition);
        }

        public void log() {

                SmartDashboard.putBoolean("At Speed", atRPM());
        }
}

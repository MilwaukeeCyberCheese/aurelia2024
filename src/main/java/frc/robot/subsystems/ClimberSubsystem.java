package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private double leftSpeed;
    private double rightSpeed;

    public ClimberSubsystem() {
        // inverted
        Constants.ClimberConstants.kLeftMotor.setInverted(Constants.ClimberConstants.kLeftInverted);
        Constants.ClimberConstants.kRightMotor.setInverted(Constants.ClimberConstants.kRightInverted);

        // idle mode
        Constants.ClimberConstants.kLeftMotor.setIdleMode(Constants.ClimberConstants.kLeftIdleMode);
        Constants.ClimberConstants.kRightMotor.setIdleMode(Constants.ClimberConstants.kRightIdleMode);

        zero();

    }

    public void setSpeeds(double speed, boolean override) {
        setLeftSpeed(speed, override);
        setRightSpeed(speed, override);
    }

    public void setLeftSpeed(double speed, boolean override) {
        if (override) {
            this.leftSpeed = speed;
        } else if (speed >= 0.0) {
            if (getLeftPosition() <= Constants.ClimberConstants.kUpperLimit) {
                this.leftSpeed = speed;
            } else {
                this.leftSpeed = 0.0;
            }
        } else {
            if (getLeftPosition() > 0.0) {
                this.leftSpeed = speed;
            } else {
                this.leftSpeed = 0.0;
            }
        }
    }

    public void setRightSpeed(double speed, boolean override) {
        if (override) {
            this.rightSpeed = speed;
        } else if (speed >= 0.0) {
            if (getRightPosition() <= Constants.ClimberConstants.kUpperLimit) {
                this.rightSpeed = speed;
            } else {
                this.rightSpeed = 0.0;
            }
        } else {
            if (getRightPosition() > 0.0) {
                this.rightSpeed = speed;
            } else {
                this.rightSpeed = 0.0;
            }
        }
    }

    public void zeroLeft() {
        Constants.ClimberConstants.kLeftMotor.getEncoder().setPosition(0);
    }

    public void zeroRight() {
        Constants.ClimberConstants.kRightMotor.getEncoder().setPosition(0);
    }

    public double getLeftPosition() {
        return Constants.ClimberConstants.kLeftEncoder.getPosition();
    }

    public double getRightPosition() {
        return Constants.ClimberConstants.kRightEncoder.getPosition();
    }

    public void zero() {
        zeroLeft();
        zeroRight();
    }

    public void periodic() {
        log();

        Constants.ClimberConstants.kLeftMotor.set(leftSpeed);
        Constants.ClimberConstants.kRightMotor.set(rightSpeed);

    }

    public void log() {
        SmartDashboard.putNumber("Left Climber Speed", Constants.ClimberConstants.kLeftMotor.get());
        SmartDashboard.putNumber("Right Climber Speed", Constants.ClimberConstants.kRightMotor.get());
        SmartDashboard.putNumber("Left Climber Position", Constants.ClimberConstants.kLeftEncoder.getPosition());
        SmartDashboard.putNumber("Right Climber Position", Constants.ClimberConstants.kRightEncoder.getPosition());

    }
}

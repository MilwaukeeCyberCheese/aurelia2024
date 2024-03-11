package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private double leftSpeed;
    private double rightSpeed;
    private double kLeftStartPos;
    private double kRightStartPos;

    public ClimberSubsystem() {
        // inverted
        Constants.ClimberConstants.kLeftMotor.setInverted(Constants.ClimberConstants.kLeftInverted);
        Constants.ClimberConstants.kRightMotor.setInverted(Constants.ClimberConstants.kRightInverted);

        //idle mode
        Constants.ClimberConstants.kLeftMotor.setIdleMode(Constants.ClimberConstants.kLeftIdleMode);
        Constants.ClimberConstants.kRightMotor.setIdleMode(Constants.ClimberConstants.kRightIdleMode);
       
        kLeftStartPos = Constants.ClimberConstants.kLeftMotorEncoder.getPosition();
        kRightStartPos = Constants.ClimberConstants.kRightMotorEncoder.getPosition();
        SmartDashboard.putNumber("Left Climber Start Position", kLeftStartPos);
        SmartDashboard.putNumber("Right Climber Start Position", kRightStartPos);

    }

    public void setSpeeds(double speed){
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }

    public void setLeftSpeed(double speed){
        if (speed >= 0.0) {
            if (Constants.ClimberConstants.kLeftMotorEncoder.getPosition() <= 
                    kLeftStartPos + Constants.ClimberConstants.kStopCounts) {
                this.leftSpeed = speed;
            } else {
                this.leftSpeed = 0.0;
            }
        } else {
            if (Constants.ClimberConstants.kLeftMotorEncoder.getPosition() > 
                    kLeftStartPos) {
                this.leftSpeed = speed;
            } else {
                this.leftSpeed = 0.0;
            }
        }
    }
  
    public void setRightSpeed(double speed){
        if (speed >= 0.0) {
            if (Constants.ClimberConstants.kRightMotorEncoder.getPosition() <= 
                    kRightStartPos + Constants.ClimberConstants.kStopCounts) {
                this.rightSpeed = speed;
            } else {
                this.rightSpeed = 0.0;
            }
        } else {
            if (Constants.ClimberConstants.kRightMotorEncoder.getPosition() > 
                    kRightStartPos) {
                this.rightSpeed = speed;
            } else {
                this.rightSpeed = 0.0;
            }
        }
    }
   

  

    public void periodic() {
        log();

        Constants.ClimberConstants.kLeftMotor.set(leftSpeed);
        Constants.ClimberConstants.kRightMotor.set(rightSpeed);

        SmartDashboard.putNumber("Left Climber Position", Constants.ClimberConstants.kLeftMotorEncoder.getPosition());
        SmartDashboard.putNumber("Right Climber Position", Constants.ClimberConstants.kRightMotorEncoder.getPosition());
    }

    public void log() {
        SmartDashboard.putNumber("Left Climber Speed", Constants.ClimberConstants.kLeftMotor.get());
        SmartDashboard.putNumber("Right Climber Speed", Constants.ClimberConstants.kRightMotor.get());
    }
}

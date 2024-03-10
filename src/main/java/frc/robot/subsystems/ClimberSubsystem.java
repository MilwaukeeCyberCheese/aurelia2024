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

        //idle mode
        Constants.ClimberConstants.kLeftMotor.setIdleMode(Constants.ClimberConstants.kLeftIdleMode);
        Constants.ClimberConstants.kRightMotor.setIdleMode(Constants.ClimberConstants.kRightIdleMode);
       
    }

    public void setSpeeds(double speed){
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }

    public void setLeftSpeed(double speed){
        this.leftSpeed = speed;
    }
  
    public void setRightSpeed(double speed){
        this.rightSpeed = speed;
    }
   

  

    public void periodic() {
        log();

        Constants.ClimberConstants.kLeftMotor.set(leftSpeed);
        Constants.ClimberConstants.kRightMotor.set(rightSpeed);
    }

    public void log() {
        SmartDashboard.putNumber("Left Climber Speed", Constants.ClimberConstants.kLeftMotor.get());
        SmartDashboard.putNumber("Right Climber Speed", Constants.ClimberConstants.kRightMotor.get());
    }
}

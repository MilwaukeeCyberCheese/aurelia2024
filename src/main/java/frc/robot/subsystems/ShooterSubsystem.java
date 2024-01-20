package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private double speed;

    public ShooterSubsystem() {
       //setup PID
       Constants.ShooterConstants.kShooterController.setP(Constants.ShooterConstants.kShooterPIDConstants.kP);
       Constants.ShooterConstants.kShooterController.setI(Constants.ShooterConstants.kShooterPIDConstants.kI);
       Constants.ShooterConstants.kShooterController.setD(Constants.ShooterConstants.kShooterPIDConstants.kD);
       Constants.ShooterConstants.kShooterController.setFeedbackDevice(Constants.ShooterConstants.kShooterEncoder);
       

       
       Constants.ShooterConstants.kShooterEncoder.setVelocityConversionFactor(Constants.ShooterConstants.kShooterConversionFactor);
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    public boolean atSpeed(){
        return Math.abs(Constants.ShooterConstants.kShooterEncoder.getVelocity() - speed) < Constants.ShooterConstants.kTolerance;
    }

    public void periodic(){
        log();
        Constants.ShooterConstants.kShooterController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    public void log(){
        SmartDashboard.putNumber("Shooter Set Speed", speed);
        SmartDashboard.putNumber("Shooter Actual Speed", Constants.ShooterConstants.kShooterEncoder.getVelocity());
        SmartDashboard.putBoolean("At Speed", atSpeed());
    }
}

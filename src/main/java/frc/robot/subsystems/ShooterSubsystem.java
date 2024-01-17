package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private PIDController shooterPID = new PIDController(Constants.ShooterConstants.kShooterPIDConstants.kP, Constants.ShooterConstants.kShooterPIDConstants.kI, Constants.ShooterConstants.kShooterPIDConstants.kD);
    private double speed;

    public ShooterSubsystem() {
        shooterPID.setTolerance(Constants.ShooterConstants.kTolerance);
        Constants.ShooterConstants.kShooterEncoder.setVelocityConversionFactor(Constants.ShooterConstants.kShooterConversionFactor);
    }

    public void setSpeed(double speed){
        this.speed = speed;
        shooterPID.setSetpoint(speed);
    }

    public void periodic(){
        log();
        Constants.ShooterConstants.kShooterMotor.set(shooterPID.calculate(Constants.ShooterConstants.kShooterEncoder.getVelocity()));
    }

    public void log(){
        SmartDashboard.putNumber("Shooter Set Speed", speed);
        SmartDashboard.putNumber("Shooter Actual Speed", Constants.ShooterConstants.kShooterEncoder.getVelocity());
    }
}

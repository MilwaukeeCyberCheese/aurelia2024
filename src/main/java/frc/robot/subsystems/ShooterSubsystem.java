package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private PIDController shooterPID = new PIDController(Constants.ShooterConstants.kShooterPIDConstants.kP, Constants.ShooterConstants.kShooterPIDConstants.kI, Constants.ShooterConstants.kShooterPIDConstants.kD);
    
    public ShooterSubsystem() {
        shooterPID.setTolerance(Constants.ShooterConstants.kTolerance);
    }

    public void setSpeed(double speed){
        
    }
}

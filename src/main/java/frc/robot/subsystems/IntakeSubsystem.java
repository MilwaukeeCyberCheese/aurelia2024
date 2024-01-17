package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private double speed;
    
    public IntakeSubsystem(){

    }

    public void periodic(){
        log();
    }

    public void setSpeed(double speed){
        this.speed = speed;
        Constants.IntakeConstants.kIntakeMotor.set(speed);
    }

    public void log(){
        SmartDashboard.putNumber("Intake Speed", speed);
    }
}

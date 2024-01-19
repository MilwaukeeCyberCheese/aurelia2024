package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private double speed;
    private PIDController m_pidController = new PIDController(Constants.IntakeConstants.kPivotPIDConstants.kP, Constants.IntakeConstants.kPivotPIDConstants.kI, Constants.IntakeConstants.kPivotPIDConstants.kD);
    
    public IntakeSubsystem(){
        //TODO
        Constants.IntakeConstants.kPivotMotor.setInverted(true);
        Constants.IntakeConstants.kIntakeMotor.setInverted(true);

        //Converts to degrees
        Constants.IntakeConstants.kPivotEncoder.setPositionConversionFactor(Constants.IntakeConstants.kPivotConversionFactor);
    }

    public void periodic(){
        log();
        Constants.IntakeConstants.kPivotMotor.set(m_pidController.calculate(Constants.IntakeConstants.kPivotEncoder.getPosition()));
    }

    public void setSpeed(double speed){
        this.speed = speed;
        Constants.IntakeConstants.kIntakeMotor.set(speed);
    }

    public void setPivotPosition(double position){
        m_pidController.setSetpoint(position);
    }

    public boolean atPosition(){
        return m_pidController.atSetpoint();
    }

    public void log(){
        SmartDashboard.putNumber("Intake Speed", speed);
    }
}

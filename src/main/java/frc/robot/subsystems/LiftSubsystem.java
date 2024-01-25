package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase{
    private double position;
    
    public LiftSubsystem(){
        //TODO
        Constants.LiftConstants.kLiftMotor.setInverted(Constants.LiftConstants.kInverted);

        //setup PID
        Constants.LiftConstants.kLiftController.setP(Constants.LiftConstants.kLiftPIDConstants.kP);
        Constants.LiftConstants.kLiftController.setI(Constants.LiftConstants.kLiftPIDConstants.kI);
        Constants.LiftConstants.kLiftController.setD(Constants.LiftConstants.kLiftPIDConstants.kD);
        Constants.LiftConstants.kLiftController.setFeedbackDevice(Constants.LiftConstants.kLiftEncoder);
        
        Constants.LiftConstants.kLiftEncoder.setPositionConversionFactor(Constants.LiftConstants.kLiftConversionFactor);
    }

    public void periodic(){
        log();
        Constants.LiftConstants.kLiftController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    
    /**
     * Set the position of the lift
     * 
     * @param position (inches)
     */
    public void setPosition(double position){
        position = MathUtil.clamp(position, Constants.LiftConstants.kLiftLimits[0], Constants.LiftConstants.kLiftLimits[1]);
        this.position = position;
    }

    /**
     * 
     * @return the position of the lift
     */
    public double getPosition(){
        return position;
    }

    /**
     * 
     * @return whether the lift is at the commanded position
     */
    public boolean atPosition(){
        return Math.abs(Constants.LiftConstants.kLiftEncoder.getPosition() - position) < Constants.LiftConstants.kTolerance;
    }

    public void log(){
        SmartDashboard.putNumber("Lift Position: ", position);
    }
}

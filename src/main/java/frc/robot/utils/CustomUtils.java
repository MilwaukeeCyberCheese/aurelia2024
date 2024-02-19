package frc.robot.utils;

import com.revrobotics.SparkPIDController;

public class CustomUtils {

    /**
     * Set the PID constants of a SparkPIDController
     *
     * @param controller
     * @param constants
     */
    public static void setSparkPID(SparkPIDController controller, PIDConstants constants) {
        controller.setP(constants.kP);
        controller.setI(constants.kI);
        controller.setD(constants.kD);
        controller.setFF(constants.kFF);
    }

    public static class WaitCommandMilli extends edu.wpi.first.wpilibj2.command.WaitCommand {

        /**
         * Instantiates a new WaitCommand
         * 
         * @param milliseconds
         */
        public WaitCommandMilli(double milliseconds) {
            super(milliseconds / 1000);
        }

    }

     
}

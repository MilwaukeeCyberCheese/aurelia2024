package frc.robot.utils;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.SparkPIDController;

public class CustomUtils {
    public static void setSparkPID(SparkPIDController controller, PIDConstants constants) {
        controller.setP(constants.kP);
        controller.setI(constants.kI);
        controller.setD(constants.kD);
    }

    public static class WaitCommandMilli extends edu.wpi.first.wpilibj2.command.WaitCommand {

        // replace generic wait command with command in millisecondsw
        public WaitCommandMilli(double milliseconds) {
            super(milliseconds / 1000);
        }

    }
}

package frc.robot.utils;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.SparkPIDController;

public class CustomUtils {
    public static void setSparkPID(SparkPIDController controller, PIDConstants constants) {
        controller.setP(constants.kP);
        controller.setI(constants.kI);
        controller.setD(constants.kD);
    }
}

package frc.robot.utils;

import com.revrobotics.SparkPIDController;

/**
 * An object that will automatically update a PID controller's constants based
 * on the SmartDashboard
 */
public class LivePIDTuner {
    private SparkPIDController controller;
    private DashboardUpdater<PIDConstants> pidConstants;

    /**
     * Instantiates a new LivePIDTuner
     * 
     * @param name
     * @param controller
     * @param constants
     */
    public LivePIDTuner(String name, SparkPIDController controller, PIDConstants constants) {
        this.controller = controller;
        pidConstants = new DashboardUpdater<PIDConstants>(name, constants);
    }

    public void update() {
        CustomUtils.setSparkPID(controller, pidConstants.update());
    }

    public PIDConstants getConstants() {
        return pidConstants.get();
    }
}

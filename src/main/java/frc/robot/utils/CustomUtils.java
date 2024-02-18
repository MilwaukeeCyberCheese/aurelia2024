package frc.robot.utils;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    /**
     * An object that will put a number to the SmartDashboard, and then update the
     * value within the object when the number on SmartDashboard changes.
     */
    public static class DashboardUpdater {
        private String key;
        private double value;

        /**
         * Instantiates a new DashboardUpdater
         * 
         * @param key
         * @param value
         */
        public DashboardUpdater(String key, double value) {
            this.key = key;
            this.value = value;
            SmartDashboard.putNumber(key, value);
        }

        public double update() {
            value = SmartDashboard.getNumber(key, value);
            return value;
        }

        public double get() {
            return value;
        }
    }

    /**
     * An object that will put a value to the SmartDashboard, and then update the
     * value within the object when the value on SmartDashboard changes.
     */
    public static class DashboardUpdaterAny <E> {
        private String key;
        private Object value;

        /**
         * Instantiates a new DashboardUpdaterAny
         * 
         * @param key
         * @param value
         */
        public DashboardUpdaterAny(String key, Object value) {
            this.key = key;
            this.value = value;
            SmartDashboard.putString(key, value.toString()); //TODO: This may need to be changed to putRaw, or further experimentation may be needed
        }

        public E update() {
            value = SmartDashboard.getString(key, value.toString());
            return (E) value;
        }

        public E get() {
            return (E) value;
        }
    }

    public static class DashboardUpdaterArray {
        private String key;
        private double[] arr;

        /**
         * Instantiates a new DashboardUpdater
         * 
         * @param key
         * @param arr
         */
        public DashboardUpdaterArray(String key, double[] value) {
            this.key = key;
            this.arr = value;
            SmartDashboard.putNumberArray(key, value);
        }

        public double[] update() {
            arr = SmartDashboard.getNumberArray(key, arr);
            return arr;
        }

        public double[] get() {
            return arr;
        }
    }

    /**
     * An object that will automatically update a PID controller's constants based
     * on the SmartDashboard
     */
    public static class LivePIDTuner {
        private SparkPIDController controller;
        private DashboardUpdaterArray pidConstants;

        /**
         * Instantiates a new LivePIDTuner
         * 
         * @param name
         * @param controller
         * @param constants
         */
        public LivePIDTuner(String name, SparkPIDController controller, PIDConstants constants) {
            this.controller = controller;
            pidConstants = new DashboardUpdaterArray(name, new double[] { constants.kP, constants.kI, constants.kD,
                    constants.kFF });
        }

        public void update() {
            double[] consts = pidConstants.update();
            setSparkPID(controller, new PIDConstants(consts[0], consts[1], consts[2], consts[3]));
        }
    }

}

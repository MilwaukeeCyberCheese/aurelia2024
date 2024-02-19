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
     * An object that will put a value to the SmartDashboard, and then update the
     * value within the object when the value on SmartDashboard changes.
     */
    public static class DashboardUpdater<E> {
        private String key;
        private Object value;

        /**
         * Instantiates a new DashboardUpdaterAny
         * 
         * @param key
         * @param value
         */
        public DashboardUpdater(String key, Object value) {
            this.key = key;
            this.value = value;
            SmartDashboard.putString(key, value.toString()); // TODO: This may need to be changed to putRaw, or further
                                                             // experimentation may be needed
        }

        @SuppressWarnings("unchecked")
        public E update() {
            value = SmartDashboard.getString(key, value.toString());
            return (E) value;
        }

        @SuppressWarnings("unchecked")
        public E get() {
            return (E) value;
        }
    }

    /**
     * Not currently working
     */
    public static class DashboardUpdaterArray {
        private String key;
        private double[] arr;

        /**
         * Instantiates a new DashboardUpdater
         * 
         * @param key
         * @param arr
         */
        public DashboardUpdaterArray(String key, double[] arr) {
            this.key = key;
            this.arr = arr;
            SmartDashboard.putNumberArray(key, arr);
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
        private DashboardUpdater<Double>[] pidConstants; //make it an ArrayList

        /**
         * Instantiates a new LivePIDTuner
         * 
         * @param name
         * @param controller
         * @param constants
         */
        public LivePIDTuner(String name, SparkPIDController controller, PIDConstants constants) {
            this.controller = controller;
            pidConstants = new DashboardUpdater[] {
                    new DashboardUpdater<Double>(name + "P", constants.kP),
                    new DashboardUpdater<Double>(name + "I", constants.kI),
                    new DashboardUpdater<Double>(name + "D", constants.kD),
                    new DashboardUpdater<Double>(name + "FF", constants.kFF)
            };
        }

        public void update() {
            for(DashboardUpdater<Double> pidConstant : pidConstants) {
                pidConstant.update();
            }
            setSparkPID(controller, new PIDConstants(pidConstants[0].get(), pidConstants[1].get(), pidConstants[2].get(), pidConstants[3].get()));
        }
    }

}

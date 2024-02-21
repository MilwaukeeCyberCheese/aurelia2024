package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An object that will put a value to the SmartDashboard, and then update the
 * value within the object when the value on SmartDashboard changes.
 */
public class DashboardUpdater<E> {
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
        SmartDashboard.putString(key, value.toString());
    }

    @SuppressWarnings("unchecked")
    public E update() {
        value = switch (value.getClass().getSimpleName()) {
            case "Double" -> SmartDashboard.getNumber(key, (Double) value);
            case "Integer" -> SmartDashboard.getNumber(key, (Integer) value);
            case "String" -> SmartDashboard.getString(key, (String) value);
            case "Boolean" -> SmartDashboard.getBoolean(key, (Boolean) value);
            case "PIDConstants" -> PIDConstants.fromString(SmartDashboard.getString(key, value.toString()));
            default -> value;
        };
        return (E) value;
    }
}

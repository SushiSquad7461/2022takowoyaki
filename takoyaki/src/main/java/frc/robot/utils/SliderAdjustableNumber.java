package frc.robot.utils;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Class for a slider adjustable number. Creates a slider, a save button, and a
 * persistent NetworkTables entry for storage.
 */
public class SliderAdjustableNumber {
    private static final String tabName = "Sliders";

    private String key;
    private double defaultValue;
    private double lastHasChangedValue = defaultValue;
    private NetworkTableEntry slider;
    private NetworkTableEntry savedValue;
    private NetworkTableEntry button;

    // contains all the slideradjustablenumbers so that we can check the buttons in
    // periodic
    private static ArrayList<SliderAdjustableNumber> allSliders = new ArrayList<SliderAdjustableNumber>();

    /**
     * Create a new TunableNumber
     * 
     * @param dashboardKey Key on dashboard
     */
    public SliderAdjustableNumber(String dashboardKey, double defaultValue, double minOffset, double maxOffset,
            double increment) {
        this.key = dashboardKey;
        this.defaultValue = defaultValue;
        this.savedValue = Shuffleboard.getTab(tabName).addPersistent(key + " saved value", defaultValue).getEntry();
        this.slider = Shuffleboard.getTab(tabName).add(key + " slider", savedValue.getDouble(defaultValue))
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", minOffset, "max", maxOffset, "block increment", increment)).getEntry();

        this.button = Shuffleboard.getTab(tabName).add("Save " + key, false).withWidget(BuiltInWidgets.kToggleButton)
                .getEntry();
        allSliders.add(this);
    }

    /**
     * Get the default value for the number that has been set
     * 
     * @return The default value
     */
    public double getDefault() {
        return defaultValue;
    }

    /**
     * Set the default value of the number
     * 
     * @param defaultValue The default value
     */
    public void saveNumber() {
        savedValue.setDouble(slider.getDouble(defaultValue));
        button.setBoolean(false);
    }

    public boolean checkButton() {
        return button.getBoolean(false);
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode
     * 
     * @return The current value
     */
    public double get() {
        return slider.getDouble(defaultValue);
    }

    // called in robot periodic
    public static void checkAllButtons() {
        for (SliderAdjustableNumber number : allSliders) {
            if (number.button.getBoolean(false)) {
                number.saveNumber();
            }
        }
    }
}

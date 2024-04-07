package frc.robot.classes;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.ColorSensorConfig;

public class ColorSensorController {
    private final DigitalInput colorSensor = new DigitalInput(0);
    private final ColorSensorConfig config;
    private ToggleHandler toggler;

    public ColorSensorController(ColorSensorConfig colorSensorConfig, ToggleHandler toggle) {
        config = colorSensorConfig;
        this.toggler = toggle;
        
    }

    public boolean isSeen() {
        SmartDashboard.putBoolean("isIntaked", colorSensor.get());
        if (this.toggler.get()) {
            return false;
        }
        return colorSensor.get();
    }

}

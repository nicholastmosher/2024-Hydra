package frc.robot.classes;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.ColorSensorConfig;

public class ColorSensorController {
    private final DigitalInput colorSensor = new DigitalInput(6);
    private final ColorSensorConfig config;

    public ColorSensorController(ColorSensorConfig colorSensorConfig) {
        config = colorSensorConfig;
        
    }

    public boolean isSeen() {
        SmartDashboard.putBoolean("isIntaked", colorSensor.get());
        return colorSensor.get();
    }

}

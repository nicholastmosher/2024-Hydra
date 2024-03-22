package frc.robot.classes;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import frc.lib.config.ColorSensorConfig;

public class ColorSensorController {
    private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private final ColorSensorConfig config;

    public ColorSensorController(ColorSensorConfig colorSensorConfig) {
        config = colorSensorConfig;
        if (colorSensor.isConnected()) {
            System.out.println("color sensor connected");
        }
        if (!colorSensor.isConnected()) {
            System.out.println("color sensor not connected");
        }
    }

    public boolean isSeen() {
        return colorSensor.getProximity() > (config.proximity - 50) && colorSensor.getProximity() < (config.proximity + 50);
    }

}

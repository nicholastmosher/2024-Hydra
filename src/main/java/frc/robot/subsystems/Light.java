package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.LightConfig;
import frc.lib.util.BlinkinPattern;
import frc.robot.classes.BlinkinLEDController;
import frc.robot.classes.ColorSensorController;

public class Light extends SubsystemBase {
    private final BlinkinLEDController blinkin1;
    private final LightConfig config;
    private final ColorSensorController colorSensor;


    public Light(LightConfig lightConfig, ColorSensorController colorSensorController) {
        config = lightConfig;
        blinkin1 = new BlinkinLEDController(config.blinkin1);
        colorSensor = colorSensorController;
        blinkin1.setPattern(BlinkinPattern.SHOT_WHITE);
        
    }

    public void lightControl() {
        if (colorSensor.isSeen()){
            blinkin1.setPattern(BlinkinPattern.SHOT_RED);
            return;
        }
        blinkin1.setPattern(BlinkinPattern.SHOT_WHITE);
    }


    public void setRed() {
        blinkin1.setPattern(BlinkinPattern.SHOT_RED);
    }

    public void setWhite() {
        blinkin1.setPattern(BlinkinPattern.SHOT_WHITE);
    }

    


}

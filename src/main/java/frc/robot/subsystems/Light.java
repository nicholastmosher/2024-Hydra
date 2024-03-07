package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.LightConfig;
import frc.robot.classes.BlinkinLEDController;

public class Light extends SubsystemBase {
    private final BlinkinLEDController blinkin;
    private final LightConfig config;


    public Light(LightConfig lightConfig) {
        blinkin = new BlinkinLEDController();
        config = lightConfig;
        
    }

    


}

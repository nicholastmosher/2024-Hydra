package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.LightConfig;

public class Light extends SubsystemBase {
    private final Spark ledControl1;

    public LightConfig config;


    public Light(LightConfig lightConfig) {
        config = lightConfig;
        ledControl1 = new Spark(config.blinkin1);
    }

    public void setColor() {
        ledControl1.set(-0.99);
//        if (color==1) {
//            ledControl1.set(config.color1);
//        }
//        if (color==2) {
//            ledControl1.set(config.color2);
//        }
    }


}

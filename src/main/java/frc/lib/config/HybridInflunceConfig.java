package frc.lib.config;

import java.util.ArrayList;

public class HybridInflunceConfig {

    public final ArrayList<Double> teleopControl;
    public final ArrayList<Double> autoControl;
    public final ArrayList<Double> intakeControl;
    public final ArrayList<Double> shootControl;




    public HybridInflunceConfig(ArrayList<Double> teleop, ArrayList<Double> auto, ArrayList<Double> intake, ArrayList<Double> shoot) {
        this.teleopControl = teleop;
        this.autoControl = auto;
        this.intakeControl = intake;
        this.shootControl = shoot;
    }
}

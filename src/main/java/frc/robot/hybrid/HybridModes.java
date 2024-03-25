package frc.robot.hybrid;

import java.util.HashMap;

public class HybridModes {
    private HashMap<String, ControlVector> modes = new HashMap<>();

    public void addMode(String name, ControlVector mode) {
        modes.put(name, mode);
    }

    /**
     *
     * @param name
     * @return
     */
    public ControlVector getMode(String name) {
        return modes.get(name);
    }

    /**
     * At t=0, return mode1, at t=1 return mode2, return interpolation between
     * @param mode1 Tha name of the first mode
     * @param mode2 The name of the second mode
     * @param t The interpolation value from 0 to 1
     * @return An interpolation of mode1 and mode2 with the given tValue
     */
    public ControlVector interpolate(String mode1, String mode2, double t) {
        ControlVector t1 = modes.get(mode1);
        ControlVector t2 = modes.get(mode2);

        ControlVector t1Scaled = t1.times(1-t);
        ControlVector t2Scaled = t2.times(t);

        return t1Scaled.plus(t2Scaled);


    }
}

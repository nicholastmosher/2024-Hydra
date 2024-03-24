package frc.robot.hybrid;

import java.util.HashMap;

public class HybridModes {
    private HashMap<String, SwerveVector> modes = new HashMap<>();

    public void addMode(String name, SwerveVector mode) {
        modes.put(name, mode);
    }

    /**
     *
     * @param name
     * @return
     */
    public SwerveVector getMode(String name) {
        return modes.get(name);
    }

    /**
     * At t=0, return mode1, at t=1 return mode2, return interpolation between
     * @param mode1 Tha name of the first mode
     * @param mode2 The name of the second mode
     * @param t The interpolation value from 0 to 1
     * @return An interpolation of mode1 and mode2 with the given tValue
     */
    public SwerveVector interpolate(String mode1, String mode2, double t) {
        SwerveVector t1 = modes.get(mode1);
        SwerveVector t2 = modes.get(mode2);

        SwerveVector t1Scaled = t1.times(1-t);
        SwerveVector t2Scaled = t2.times(t);

        return t1Scaled.plus(t2Scaled);


    }
}

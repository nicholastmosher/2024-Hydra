package frc.robot.classes;

public class ToggleHandler {
    private boolean state = false;

    public void toggle() {
        state = !state;
    }

    public boolean get() {
        return state;
    }

}

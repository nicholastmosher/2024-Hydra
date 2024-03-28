package frc.robot.classes;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ToggleHandler {
    private boolean state;

    public ToggleHandler() {
        this.state = false;
    }

    public void toggle() {
        state = !state;
    }

    public boolean get() {
        return state;
    }

}

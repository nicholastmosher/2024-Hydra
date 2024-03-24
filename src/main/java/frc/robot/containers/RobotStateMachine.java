package frc.robot.containers;

public class RobotStateMachine {
    private boolean isIntaking = false;
    public boolean isIntaking() {
        return isIntaking;
    }
    public void toggleIntaking() {
        isIntaking = !isIntaking;
    }
    public void setIntaking(boolean state) {
        isIntaking = state;
    }
}

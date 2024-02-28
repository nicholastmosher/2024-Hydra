package frc.robot.containers;

public class RobotStateMachine {
    private boolean isArmRaised = false;
    private boolean isHoldingNote = false;
    private boolean isElevatorRaised = false;

    public RobotStateMachine(){}
    public boolean isShootingAllowed(){
        return isHoldingNote && !isElevatorRaised;
    }

    public boolean isRaisingArmAllowed(){
        return isHoldingNote && !isElevatorRaised;
    }

    public boolean isRaisingElevatorAllowed(){
        return !isArmRaised;
    }

    public boolean isIntakeAllowed(){
        return !isHoldingNote && !isArmRaised;
    }

    public void setNoteState(boolean isHolding){
        this.isHoldingNote = isHolding;
    }

    public void setElevatorState(boolean isElevatorRaised){
        this.isElevatorRaised = isElevatorRaised;
    }

    public void setArmState(boolean isArmRaised){
        this.isArmRaised = isArmRaised;
    }
}

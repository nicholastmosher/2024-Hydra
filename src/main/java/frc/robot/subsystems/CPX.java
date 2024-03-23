package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CPX extends SubsystemBase {
    private final DigitalOutput output;

    public CPX(int dioChannel) {
        output = new DigitalOutput(dioChannel);
    }

    public void set(boolean status){
        output.set(status);
    }
}
package frc.lib.config;

import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberConfig {

    public final int rightClimberMotorID;
    public final int leftClimberMotorID;
    public final double climberSpeed;
    public final double downclimberSpeed;
    public final DigitalInput rightMotorLimitSwitch;
    public final DigitalInput leftMotorLimitSwitch;


    public ClimberConfig(int rightMotorID, int leftMotorID, double speed, double reversespeed, DigitalInput rightlimit, DigitalInput leftlimit) {
        this.rightClimberMotorID = rightMotorID;
        this.leftClimberMotorID = leftMotorID;
        this.climberSpeed = speed;
        this.downclimberSpeed = reversespeed;
        this.rightMotorLimitSwitch = rightlimit;
        this.leftMotorLimitSwitch = leftlimit;
    }
}

package frc.lib.config;

public class IntakeConfig {
    public final int intakeMotorId;
    public final double intakeMotorSpeed;
    public final int proximity;

    public IntakeConfig(int motorId, double motorSpeed, int prox) {
        this.intakeMotorId = motorId;
        this.intakeMotorSpeed = motorSpeed;
        this.proximity = prox;
    }
}

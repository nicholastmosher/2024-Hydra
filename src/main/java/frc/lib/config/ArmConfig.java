package frc.lib.config;

public class ArmConfig {
    public final int rightMotorId;
    public final int leftMotorId;

    public final PidConfig pid;

    public final double shootAngle;
    public final double intakeAngle;
    public final double ampAngle;

    public ArmConfig(int rightMotor, int leftMotor, PidConfig pid) {
        this(
                rightMotor,
                leftMotor,
                pid,
                0.0, // Default shoot angle
                90.0, // Default intake angle
                60.0 // Default amp angle
        );
    }

    public ArmConfig(int rightMotor, int leftMotor, PidConfig pid, double shootAngle, double intakeAngle, double ampAngle) {
        this.rightMotorId = rightMotor;
        this.leftMotorId = leftMotor;
        this.pid = pid;
        this.shootAngle = shootAngle;
        this.intakeAngle = intakeAngle;
        this.ampAngle = ampAngle;
    }
}

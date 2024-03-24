package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;

public class ArmConfig {
    public final int rightMotorId;
    public final int leftMotorId;

    public final PidConfig pid;

    public final double shootAngle;
    public final double intakeAngle;
    public final double ampAngle;
    public final double zeroOffset;

    public final double positionScalingFactor;

    public final DigitalInput armLimitSwitch;

    public ArmConfig(int rightMotor, int leftMotor, PidConfig pid, double scalingFactor) {
        this(
                rightMotor,
                leftMotor,
                pid,
                0.0, // Default shoot angle
                90.0, // Default intake angle
                60.0, // Default amp angle
                scalingFactor,
                0.0,
                new DigitalInput(2)
        );
    }

    public ArmConfig(int rightMotor, int leftMotor, PidConfig pid, double shootAngle, double intakeAngle, double ampAngle, double scalingFactor, double offset, DigitalInput limitSwitch) {
        this.rightMotorId = rightMotor;
        this.leftMotorId = leftMotor;
        this.pid = pid;
        this.shootAngle = shootAngle;
        this.intakeAngle = intakeAngle;
        this.ampAngle = ampAngle;
        this.positionScalingFactor = scalingFactor;
        this.zeroOffset = offset;
        this.armLimitSwitch=limitSwitch;
    }
}

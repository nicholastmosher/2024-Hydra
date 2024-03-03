package frc.lib.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;

public class ArmConfig {
    public final int rightMotorId;
    public final int leftMotorId;

    public final PidConfig pid;

    public final Rotation2d shootAngle;
    public final Rotation2d intakeAngle;
    public final Rotation2d ampAngle;
    public final Rotation2d zeroOffset;

    public final double positionScalingFactor;

    public final DigitalInput armLimitSwitch;

    public ArmConfig(int rightMotor, int leftMotor, PidConfig pid, double scalingFactor) {
        this(
                rightMotor,
                leftMotor,
                pid,
                Rotation2d.fromDegrees(0.0), // Default shoot angle
                Rotation2d.fromDegrees(90.0), // Default intake angle
                Rotation2d.fromDegrees(60.0), // Default amp angle
                scalingFactor,
                Rotation2d.fromDegrees(0.0),
                new DigitalInput(2)
        );
    }

    public ArmConfig(int rightMotor, int leftMotor, PidConfig pid, Rotation2d shootAngle, Rotation2d intakeAngle, Rotation2d ampAngle, double scalingFactor, Rotation2d offset, DigitalInput limitSwitch) {
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

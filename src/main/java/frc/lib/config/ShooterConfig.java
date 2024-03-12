package frc.lib.config;

public class ShooterConfig {
    public final int shooterTopMotor;
    public final int shooterBottomMotor;
    public final double shooterSpeed;
    public final double targetVelocity;

    public ShooterConfig(int topMotor, int bottomMotor, double shooterSpeed, double velocity) {
        this.shooterTopMotor = topMotor;
        this.shooterBottomMotor = bottomMotor;
        this.shooterSpeed = shooterSpeed;
        this.targetVelocity = velocity;

    }
}

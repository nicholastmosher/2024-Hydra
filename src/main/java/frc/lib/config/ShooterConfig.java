package frc.lib.config;

public class ShooterConfig {
    public final int shooterTopMotor;
    public final int shooterBottomMotor;
    public final int shooterIntakeMotor;

    public final PidConfig pid;

    public final double shooterIntakeSpeed;
    public final double indexerFeedSpeed;
    public final double shooterSpeed;
    public final double targetVelocity;

    public ShooterConfig(int topMotor, int bottomMotor, int intakeMotor, PidConfig pid, double intakeSpeed, double indexerSpeed, double shooterSpeed, double velocity) {
        this.shooterTopMotor = topMotor;
        this.shooterBottomMotor = bottomMotor;
        this.shooterIntakeMotor=intakeMotor;
        this.pid = pid;
        this.shooterIntakeSpeed = intakeSpeed;
        this.indexerFeedSpeed = indexerSpeed;
        this.shooterSpeed = shooterSpeed;
        this.targetVelocity = velocity;

    }
}

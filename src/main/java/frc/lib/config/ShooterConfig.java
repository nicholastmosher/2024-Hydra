package frc.lib.config;

public class ShooterConfig {
    public final int shooterTopMotor;
    public final int shooterBottomMotor;
    public final int shooterIntakeMotor;

    public final PidConfig pid;

    public final double shooterIntakeSpeed;
    public final double shooterSpeed;

    public ShooterConfig(int topMotor, int bottomMotor, int intakeMotor, PidConfig pid, double intakeSpeed, double shooterSpeed) {
        this.shooterTopMotor = topMotor;
        this.shooterBottomMotor = bottomMotor;
        this.shooterIntakeMotor=intakeMotor;
        this.pid = pid;
        this.shooterIntakeSpeed = intakeSpeed;
        this.shooterSpeed = shooterSpeed;

    }
}

package frc.lib.config;

public class IndexerConfig {
    public final int shooterIntakeMotor;
    public final double shooterIntakeSpeed;
    public final double indexerFeedSpeed;

    public IndexerConfig(int intakeMotor, double intakeSpeed, double indexerSpeed) {
        this.shooterIntakeMotor=intakeMotor;
        this.shooterIntakeSpeed = intakeSpeed;
        this.indexerFeedSpeed = indexerSpeed;

    }
}

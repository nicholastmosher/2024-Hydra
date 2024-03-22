package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DashboardConfig;
import frc.lib.config.IndexerConfig;
import frc.lib.config.ShooterConfig;

public class Indexer extends SubsystemBase {
    private final CANSparkMax indexerMotor;

    public final IndexerConfig config;


    public Indexer(IndexerConfig config) {
        this.config = config;
        indexerMotor = new CANSparkMax(this.config.shooterIntakeMotor, CANSparkLowLevel.MotorType.kBrushless);
    }

    public void indexNote() {
        indexerMotor.set(config.shooterIntakeSpeed);
    }
    public void feedNote() {
        indexerMotor.set(-0.5);
    }
    public void stopIndexer() {
        indexerMotor.stopMotor();
    }
    public void sendBack() {
        indexerMotor.set(0.15);
    }

    public boolean isIndexerStopped() {
        if (indexerMotor.getEncoder().getVelocity() ==0) {
            return true;
        }
        return false;
    }

}

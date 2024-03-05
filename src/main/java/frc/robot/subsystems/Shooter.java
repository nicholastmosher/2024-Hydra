package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DashboardConfig;
import frc.lib.config.ShooterConfig;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex shooterTopMotor;
    private final CANSparkFlex shooterBottomMotor;
    private final CANSparkMax shooterIntakeMotor;

    public final ShooterConfig config;
    //public final DashboardConfig dashboardConfig;

    public Shooter(ShooterConfig config) {
        this.config = config;
        //this.dashboardConfig = dashboardConfig;

        shooterTopMotor = new CANSparkFlex(this.config.shooterTopMotor, CANSparkLowLevel.MotorType.kBrushless);
        SparkPIDController shooterPID = shooterTopMotor.getPIDController();
        shooterPID.setP(1.0);

        shooterBottomMotor = new CANSparkFlex(this.config.shooterBottomMotor, CANSparkLowLevel.MotorType.kBrushless);
        shooterBottomMotor.follow(shooterTopMotor);

        shooterIntakeMotor = new CANSparkMax(this.config.shooterIntakeMotor, CANSparkLowLevel.MotorType.kBrushless);
    }

    public void indexNote() {
        shooterIntakeMotor.set(-0.6);
    }
    public void feedNote() {
        shooterIntakeMotor.set(config.indexerFeedSpeed);

//        shooterIntakeMotor.getEncoder().setPosition(0);
//        shooterIntakeMotor.getPIDController().setReference(25, CANSparkBase.ControlType.kPosition);
    }

    public void startShooter() {
        shooterTopMotor.set(this.config.shooterSpeed);
    }

    public void stop() {
        shooterTopMotor.stopMotor();
        shooterIntakeMotor.stopMotor();
    }
    public void stopFeed() {
        shooterIntakeMotor.stopMotor();
    }
    public void stopShoot() {
        shooterTopMotor.stopMotor();
    }

    public boolean isRevved() {
        return shooterTopMotor.getEncoder().getVelocity() > (config.targetVelocity - 200) && shooterTopMotor.getEncoder().getVelocity() < (config.targetVelocity + 200);
    }

    public void dashboardPeriodic(){
        //SmartDashboard.putNumber(dashboardConfig.SHOOTER_TOP_MOTOR_VELOCITY, shooterTopMotor.getEncoder().getVelocity());
        //SmartDashboard.putNumber(dashboardConfig.SHOOTER_BOTTOM_MOTOR_VELOCITY, shooterBottomMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("shooterVelocity", shooterTopMotor.getEncoder().getVelocity());
        SmartDashboard.putBoolean("is revved", isRevved());
    }

}

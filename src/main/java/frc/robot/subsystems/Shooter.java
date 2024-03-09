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

    private final SparkPIDController shooterPID;

    //private boolean IsRevved;

    public final ShooterConfig config;
    //public final DashboardConfig dashboardConfig;

    public Shooter(ShooterConfig config) {
        this.config = config;
        //this.dashboardConfig = dashboardConfig;

        shooterTopMotor = new CANSparkFlex(this.config.shooterTopMotor, CANSparkLowLevel.MotorType.kBrushless);
        shooterPID = shooterTopMotor.getPIDController();
        shooterPID.setP(config.pid.kP);

        shooterBottomMotor = new CANSparkFlex(this.config.shooterBottomMotor, CANSparkLowLevel.MotorType.kBrushless);
        shooterBottomMotor.follow(shooterTopMotor);

        shooterIntakeMotor = new CANSparkMax(this.config.shooterIntakeMotor, CANSparkLowLevel.MotorType.kBrushless);


    }

    public void indexNote() {
        shooterIntakeMotor.set(-0.6);
    }
    public void feedNote() {
        shooterIntakeMotor.set(config.indexerFeedSpeed);
    }

    public void startShooter() {
//        shooterTopMotor.set(this.config.shooterSpeed);
        shooterTopMotor.set(0.5);
        //shooterPID.setReference(config.targetVelocity, CANSparkBase.ControlType.kVelocity);
    }

    public void stop() {
        shooterTopMotor.stopMotor();
        shooterIntakeMotor.stopMotor();
    }
    public void stopFeed() {
        shooterIntakeMotor.set(0);
    }
    public void stopShoot() {
        shooterTopMotor.stopMotor();
    }

    public void sendBack() {
        shooterIntakeMotor.set(0.1);
    }

    public void sendBackShooter() {
        shooterTopMotor.set(-0.05);
    }

    public boolean isRevved() {
        return true;//shooterTopMotor.getEncoder().getVelocity() > (config.targetVelocity - 200) && shooterTopMotor.getEncoder().getVelocity() < (config.targetVelocity + 200);
    }

//    public void setRevved() {
//
//    }

    public boolean isIndexerStopped() {
        if (shooterIntakeMotor.getEncoder().getVelocity() ==0) {
            return true;
        }
        return false;
    }
    public boolean isShooterStopped() {
        if (shooterTopMotor.getEncoder().getVelocity() ==0) {
            return true;
        }
        return false;
    }

    public void dashboardPeriodic(){
        //SmartDashboard.putNumber(dashboardConfig.SHOOTER_TOP_MOTOR_VELOCITY, shooterTopMotor.getEncoder().getVelocity());
        //SmartDashboard.putNumber(dashboardConfig.SHOOTER_BOTTOM_MOTOR_VELOCITY, shooterBottomMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("shooterVelocity", shooterTopMotor.getEncoder().getVelocity());
        // SmartDashboard.putBoolean("is revved", isRevved());
    }

}
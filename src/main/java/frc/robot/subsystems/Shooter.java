package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.DashboardConfig;
import frc.lib.config.ShooterConfig;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex shooterTopMotor;
    private final CANSparkFlex shooterBottomMotor;

    public final ShooterConfig config;
    public Shooter(ShooterConfig config) {
        this.config = config;
        //this.dashboardConfig = dashboardConfig;

        shooterTopMotor = new CANSparkFlex(this.config.shooterTopMotor, CANSparkLowLevel.MotorType.kBrushless);

        shooterBottomMotor = new CANSparkFlex(this.config.shooterBottomMotor, CANSparkLowLevel.MotorType.kBrushless);
        shooterBottomMotor.follow(shooterTopMotor);


    }

    public void startShooter() {
        shooterTopMotor.set(0.5);
    }

    public void stopShooter() {
        shooterTopMotor.stopMotor();
    }
    public void stopShoot() {
        shooterTopMotor.stopMotor();
    }

    public void sendBackShooter() {
        shooterTopMotor.set(-0.05);
    }

    public boolean isRevved() {
        return true;//shooterTopMotor.getEncoder().getVelocity() > (config.targetVelocity - 200) && shooterTopMotor.getEncoder().getVelocity() < (config.targetVelocity + 200);
    }

    public boolean isShooterStopped() {
        if (shooterTopMotor.getEncoder().getVelocity() ==0) {
            return true;
        }
        return false;
    }


}

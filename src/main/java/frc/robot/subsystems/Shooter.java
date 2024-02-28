package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
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
        //shooterIntakePID.setReference(Constants.Shooter.shooterIntakeSpeed, CANSparkBase.ControlType.kVelocity);
        shooterIntakeMotor.set(-0.8);
    }

    public void shootNote() {
        //shooterPID.setReference(1000, CANSparkBase.ControlType.kVelocity);
        shooterIntakeMotor.set(this.config.shooterIntakeSpeed);
        shooterTopMotor.set(this.config.shooterSpeed);
    }

    public void stop() {
        shooterTopMotor.stopMotor();
        shooterIntakeMotor.stopMotor();
    }

    public void dashboardPeriodic(){
        //SmartDashboard.putNumber(dashboardConfig.SHOOTER_TOP_MOTOR_VELOCITY, shooterTopMotor.getEncoder().getVelocity());
        //SmartDashboard.putNumber(dashboardConfig.SHOOTER_BOTTOM_MOTOR_VELOCITY, shooterBottomMotor.getEncoder().getVelocity());

    }

}

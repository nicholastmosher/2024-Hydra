package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.ShooterConfig;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex shooterTopMotor;
    private final CANSparkFlex shooterBottomMotor;
    private final CANSparkMax shooterIntakeMotor;

    public ShooterConfig config;

    public Shooter(ShooterConfig config) {
        this.config = config;
        shooterTopMotor = new CANSparkFlex(this.config.shooterTopMotor, CANSparkLowLevel.MotorType.kBrushless);
        SparkPIDController shooterPID = shooterTopMotor.getPIDController();
        shooterPID.setP(1.0);
        configShooterMotor();

        shooterBottomMotor = new CANSparkFlex(this.config.shooterBottomMotor, CANSparkLowLevel.MotorType.kBrushless);
        configShooterFollowerMotor();

        shooterIntakeMotor = new CANSparkMax(this.config.shooterIntakeMotor, CANSparkLowLevel.MotorType.kBrushless);
        configShooterIntakeMotor();

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

    private void configShooterMotor() {

    }

    private void configShooterFollowerMotor() {
        shooterBottomMotor.follow(shooterTopMotor);
    }

    private void configShooterIntakeMotor() {

    }
}

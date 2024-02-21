package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex shooterMotor;
    private final CANSparkFlex shooterFollowerMotor;
    private final CANSparkMax shooterIntakeMotor;

    public Shooter() {
        shooterMotor = new CANSparkFlex(Constants.Shooter.shooterMotor, CANSparkLowLevel.MotorType.kBrushless);
        SparkPIDController shooterPID = shooterMotor.getPIDController();
        shooterPID.setP(1.0);
        configShooterMotor();

        shooterFollowerMotor = new CANSparkFlex(Constants.Shooter.shooterFollowerMotor, CANSparkLowLevel.MotorType.kBrushless);
        configShooterFollowerMotor();

        shooterIntakeMotor = new CANSparkMax(Constants.Shooter.shooterIntakeMotor, CANSparkLowLevel.MotorType.kBrushless);
        configShooterIntakeMotor();

    }

    public void indexNote() {
        //shooterIntakePID.setReference(Constants.Shooter.shooterIntakeSpeed, CANSparkBase.ControlType.kVelocity);
        shooterIntakeMotor.set(-0.8);
    }

    public void shootNote() {
        //shooterPID.setReference(1000, CANSparkBase.ControlType.kVelocity);
        shooterIntakeMotor.set(Constants.Shooter.shooterIntakeSpeed);
        shooterMotor.set(Constants.Shooter.shooterSpeed);
    }

    public void stop() {
        shooterMotor.stopMotor();
        shooterIntakeMotor.stopMotor();
    }

    private void configShooterMotor() {

    }

    private void configShooterFollowerMotor() {
        shooterFollowerMotor.follow(shooterMotor);
    }

    private void configShooterIntakeMotor() {

    }
}

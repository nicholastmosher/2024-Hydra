package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.Constants;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex shooterMotor;
    private final CANSparkFlex shooterFollowerMotor;
    private final CANSparkMax shooterIntakeMotor;

    private final SparkPIDController shooterPID;
    private final SparkPIDController shooterIntakePID;

    public Shooter() {
        shooterMotor = new CANSparkFlex(Constants.Shooter.shooterMotor, CANSparkLowLevel.MotorType.kBrushless);
        shooterPID = shooterMotor.getPIDController();
        shooterPID.setP(1.0);
        configShooterMotor();

        shooterFollowerMotor = new CANSparkFlex(Constants.Shooter.shooterFollowerMotor, CANSparkLowLevel.MotorType.kBrushless);
        configShooterFollowerMotor();

        shooterIntakeMotor = new CANSparkMax(Constants.Shooter.shooterIntakeMotor, CANSparkLowLevel.MotorType.kBrushless);
        shooterIntakePID = shooterIntakeMotor.getPIDController();
        configShooterIntakeMotor();

    }

    public void indexNote() {
        //shooterIntakePID.setReference(Constants.Shooter.shooterIntakeSpeed, CANSparkBase.ControlType.kVelocity);
        shooterIntakeMotor.set(-0.8);
    }

    public void shootNote() {
        //shooterPID.setReference(1000, CANSparkBase.ControlType.kVelocity);
        shooterMotor.set(Constants.Shooter.shooterSpeed);
    }

    public void stop() {shooterMotor.stopMotor();}

    public void start () {
        shooterMotor.set(0.2);
    }

    private void configShooterMotor() {

    }

    private void configShooterFollowerMotor() {
        shooterFollowerMotor.follow(shooterMotor);
    }

    private void configShooterIntakeMotor() {

    }
}

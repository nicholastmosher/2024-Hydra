package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants;

public class Arm extends SubsystemBase {
    private final CANSparkMax armMotor;
    private final CANSparkMax armFollowerMotor;

    private final SparkPIDController armPID;

    private final AbsoluteEncoder armEncoder;

    public Arm() {
        armMotor = new CANSparkMax(Constants.Arm.armMotor, CANSparkLowLevel.MotorType.kBrushless);
        armPID = armMotor.getPIDController();
        configArmMotor();

        armFollowerMotor = new CANSparkMax(Constants.Arm.armFollowerMotor, CANSparkLowLevel.MotorType.kBrushless);
        configFollowerMotor();

        armEncoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    }

    public void setAngle(Rotation2d angle) {
        armPID.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
    }

    public boolean endCondition(Rotation2d angle) {
        return armEncoder.getPosition() > (angle.getDegrees() + 1) && armEncoder.getPosition() < (angle.getDegrees() - 1);
    }

    public void stopSet() {
        armMotor.stopMotor();
    }

    private void configArmMotor() {
        armPID.setOutputRange(Constants.Arm.minAngle, Constants.Arm.maxAngle);
        armPID.setFeedbackDevice(armEncoder);
    }

    private void configFollowerMotor() {
        armFollowerMotor.follow(armMotor, true);
    }

    private Rotation2d getPosition() {
        return Rotation2d.fromRotations(armEncoder.getPosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", getPosition().getDegrees());
    }
}

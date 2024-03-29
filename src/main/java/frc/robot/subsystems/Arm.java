package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.ArmConfig;
import frc.robot.hybrid.BlendedControl;

public class Arm extends SubsystemBase {
    private final CANSparkMax armLeftMotor;
    private final CANSparkMax armRightMotor;

    private final AbsoluteEncoder armEncoder;
    private final DigitalInput armLimitSwitch;
    public final ArmConfig config;

    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);
    private PIDController armPIDController;

    private final double minAngleToShoot =0;
    private final double maxAngleToShoot =0;



    public Arm(ArmConfig config) {
        this.config = config;

        armLeftMotor = new CANSparkMax(this.config.leftMotorId, CANSparkLowLevel.MotorType.kBrushless);
        armEncoder = armLeftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        armEncoder.setPositionConversionFactor(config.positionScalingFactor);
        armEncoder.setZeroOffset(163.32+180);

        armRightMotor = new CANSparkMax(this.config.rightMotorId, CANSparkLowLevel.MotorType.kBrushless);
        armRightMotor.follow(armLeftMotor, true);

        armLimitSwitch = config.armLimitSwitch;

        armPIDController = new PIDController(.03, 0.00, 0);
    }

    /**
     * Sets the target angle of the arm. This will determine where
     * the PID controller tries to move the arm.
     */
    public void setTargetAngle(Rotation2d angle) {
        this.targetAngle = angle;
    }

    /**
     * Returns a power from -1.0 to 1.0 to drive the arm.
     * This is calculated by a PID controller that wants
     * to reach the latest target angle.
     */
    public double getArmPowerToTarget() {
        Rotation2d encoder = Rotation2d.fromRotations(armEncoder.getPosition());
        return armPIDController.calculate(encoder.getDegrees(), this.targetAngle.getDegrees());
    }

    public void moveArm(double input) {



        if (input < 0) {
            if (isFullLower()) {
                armLeftMotor.stopMotor();
                return;
            }

            armLeftMotor.set(input);
            return;
        }
        if (input > 0) {
            // if (armEncoder.getPosition() > (config.ampAngle - 0.05) && armEncoder.getPosition() < (config.ampAngle + 0.05)) {
            //     armLeftMotor.stopMotor();
            //     return;
            // }

            armLeftMotor.set(input);
            return;
        }
        armLeftMotor.set(0);

    }

    public double percentRaised() {
        double diff = config.ampAngle-config.intakeAngle;

        double m = 1/diff;

        return (armEncoder.getPosition()-config.intakeAngle)/diff;
    }

    public boolean endCondition(double angle) {
        return armEncoder.getPosition() > (config.ampAngle - 0.05) && armEncoder.getPosition() < (config.ampAngle + 0.05);
    }

    public void stopMotor() {
        armLeftMotor.stopMotor();
        armRightMotor.stopMotor();
        System.out.println("Stop All Arm Movement");
    }

    private Rotation2d getPosition() {
        return Rotation2d.fromRotations(armEncoder.getPosition());
    }

    private boolean isFullLower() {
        return !armLimitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("armAngle", armEncoder.getPosition());

    }
}

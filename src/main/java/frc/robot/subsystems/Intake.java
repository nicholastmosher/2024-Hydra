package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Constants;
import frc.lib.config.IntakeConfig;

public class Intake extends SubsystemBase {
    private final I2C.Port colorSensorPort = I2C.Port.kOnboard;
    private final CANSparkMax intakeMotor;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(colorSensorPort);

    public final IntakeConfig config;

    public Intake(IntakeConfig config) {
        this.config = config;
        intakeMotor = new CANSparkMax(this.config.intakeMotorId, CANSparkLowLevel.MotorType.kBrushless);
    }

    public void setIntakeMotor(boolean invert) {
        if (!invert) {
            intakeMotor.set(this.config.intakeMotorSpeed);
            return;
        }
        intakeMotor.set(-this.config.intakeMotorSpeed);
    }

    public boolean endCondition() {
        return colorSensor.getProximity() > (config.proximity - 50) && colorSensor.getProximity() < (config.proximity + 50);

    }

    public void stopIntakeMotor() {
        intakeMotor.set(0);
    }

    public boolean isIntakeStopped() {
        if (intakeMotor.getEncoder().getVelocity() ==0) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Is Intaked", endCondition());
        // SmartDashboard.putBoolean("ColorSensorStatusConnected", colorSensor.isConnected());
        // SmartDashboard.putNumber("SensorDist", colorSensor.getProximity());
    }

}

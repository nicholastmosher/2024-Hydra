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
    private final ColorMatch colorMatcher = new ColorMatch();

    public final IntakeConfig config;

    public Intake(IntakeConfig config) {
        this.config = config;
        intakeMotor = new CANSparkMax(this.config.intakeMotorId, CANSparkLowLevel.MotorType.kBrushless);
        Color orangeColor = new Color(0.561, 0.232, 0.114);
        colorMatcher.addColorMatch(orangeColor);
    }

    public void setIntakeMotor() {
        intakeMotor.set(this.config.intakeMotorSpeed);
    }

    public boolean endCondition() {
        ColorMatchResult color = colorMatcher.matchClosestColor(colorSensor.getColor());
        return color.color == Color.kOrange;
    }

    public void stopIntakeMotor() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is Intaked", endCondition());
    }

}

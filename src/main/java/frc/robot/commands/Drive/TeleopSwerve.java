package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.krakenTalonConstants;
import frc.robot.classes.Limelight.LimelightController;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier y;
    private DoubleSupplier x;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier autoAimSup;
    private BooleanSupplier autoIntakeAlignSup;
    private LimelightController shootLimelight;
    private LimelightController intakeLimelight;
    private PIDController shootPID;
    private PIDController intakePID;

    public TeleopSwerve(
            Swerve s_Swerve,
            DoubleSupplier x,
            DoubleSupplier y,
            DoubleSupplier rotationSup,
            BooleanSupplier robotCentricSup,
            BooleanSupplier autoAimSup,
            BooleanSupplier autoIntakeAlignSup,
            LimelightController shootLimelight,
            LimelightController intakeLimelight
    ) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.y = y;
        this.x = x;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.autoAimSup = autoAimSup;
        this.autoIntakeAlignSup = autoIntakeAlignSup;
        this.shootLimelight = shootLimelight;
        this.intakeLimelight = intakeLimelight;
        this.shootPID = new PIDController(1, 0, 0);
        this.intakePID = new PIDController(1, 0, 0);
    }

    @Override
    public void execute() {
        double maxSpeed;

        /* Get Values, Deadband*/
        double y = MathUtil.applyDeadband(this.y.getAsDouble(), krakenTalonConstants.stickDeadband);
        double x = MathUtil.applyDeadband(this.x.getAsDouble(), krakenTalonConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), krakenTalonConstants.stickDeadband);
        double shootOrient = shootPID.calculate(shootLimelight.getYawToSpeaker(), 0);
        double intakeOrient = intakePID.calculate(intakeLimelight.getYawToNote(), 0);
        if (autoAimSup.getAsBoolean() && !autoIntakeAlignSup.getAsBoolean()) {
            rotationVal = shootOrient;
        }
        if (autoIntakeAlignSup.getAsBoolean() && !autoAimSup.getAsBoolean()) {
            rotationVal = intakeOrient;
        }

        Translation2d translation = new Translation2d(x*krakenTalonConstants.Swerve.maxSpeed, y*krakenTalonConstants.Swerve.maxSpeed);

        /* Drive */
        s_Swerve.drive(
                translation,
                rotationVal * krakenTalonConstants.Swerve.maxAngularVelocity,
                !robotCentricSup.getAsBoolean(),
                true
        );

        SmartDashboard.putNumber("shootorient",shootOrient);
        SmartDashboard.putNumber("orienttoNote", intakeOrient);
//        SmartDashboard.putNumber("", );
    }
}
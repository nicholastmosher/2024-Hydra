package frc.robot.commands.Drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.lib.Constants;
import frc.lib.krakentalon.krakenTalonConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class AutoSwerve extends Command {
    private Swerve s_Swerve;
    private double y;
    private double x;
    private double rotationSup;
    private boolean robotCentricSup;

    SlewRateLimiter Xfilter = new SlewRateLimiter(0.5);
    SlewRateLimiter Yfilter = new SlewRateLimiter(0.5);
    SlewRateLimiter Anglefilter = new SlewRateLimiter(0.5);

    public AutoSwerve(Swerve s_Swerve, double x, double y, double rotationSup, boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.y = y;
        this.x = x;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        double maxSpeed;

        /* Get Values, Deadband*/
        double y = MathUtil.applyDeadband(this.y, krakenTalonConstants.stickDeadband);
        double x = MathUtil.applyDeadband(this.x, krakenTalonConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup, krakenTalonConstants.stickDeadband);

        Translation2d slewedTranslation = new Translation2d(Xfilter.calculate(x*krakenTalonConstants.Swerve.maxSpeed), Yfilter.calculate(y*krakenTalonConstants.Swerve.maxSpeed));
        Translation2d translation = new Translation2d(x*krakenTalonConstants.Swerve.maxSpeed, y*krakenTalonConstants.Swerve.maxSpeed);

        /* Drive */
        s_Swerve.drive(
                translation,
                rotationVal * krakenTalonConstants.Swerve.maxAngularVelocity,
                !robotCentricSup,
                true
        );
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 0.0, false, true);
    }
}
package frc.robot.commands.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hybrid.BlendedControl;
import frc.robot.hybrid.ControlVector;
import frc.robot.subsystems.Swerve;

public class HybridSwerve extends Command {
    private final Swerve s_Swerve;
    private final BlendedControl swerveInputs;

    public HybridSwerve(
            Swerve s_Swerve,
            BlendedControl inputs
    ) {
        this.s_Swerve = s_Swerve;
        this.swerveInputs = inputs;

        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        ControlVector driveOutputs = swerveInputs.solve();
        ChassisSpeeds driveSpeeds = driveOutputs.calculateChassisSpeeds(s_Swerve.getHeading());
        s_Swerve.driveChassisSpeeds(driveSpeeds, true);
    }
}
package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Swerve;

import frc.lib.krakentalon.krakenTalonConstants;


public class TrajectoryFollowerCommands {

    private Swerve s_swerve;
    private HolonomicDriveController controller;
    private HolonomicPathFollowerConfig config;
    private boolean isBlue;
    public TrajectoryFollowerCommands(Swerve subsystem, boolean isBlueAlliance) {
        this.isBlue = isBlueAlliance;
        this.config = new HolonomicPathFollowerConfig(
                new PIDConstants(100, 1 ,0),
                new PIDConstants(100, 1,0),
                krakenTalonConstants.Swerve.maxSpeed,
                krakenTalonConstants.Swerve.swerveRadius,
                new ReplanningConfig(false, false)
                );
        s_swerve = subsystem;



    }

    public FollowPathHolonomic followPath(String chosenPath) {

        PathPlannerPath path = PathPlannerPath.fromPathFile(chosenPath);

        return new FollowPathHolonomic(
                path,
                s_swerve::getPose,
                s_swerve::getLatestSpeeds,
                (ChassisSpeeds chassisSpeeds) -> s_swerve.driveChassisSpeeds(chassisSpeeds, true),
                this.config,
                () -> this.isBlue,
                s_swerve


        );
    }
}

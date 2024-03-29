package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.CommandGroups.ShootCommands.PrepareShootCommandGroup;
import frc.robot.subsystems.*;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.config.krakenTalonConstants;
import frc.robot.commands.CommandGroups.IntakeCommands.IntakeCommandGroup;
import frc.robot.commands.Drive.AutoSwerve;
import frc.robot.commands.Shooter.RevShooter;


public class AutoCommands {
    private final Swerve swerveSubsystem;
    private final Arm armSubsystem;
    private final Indexer indexerSubsystem;
    private final Shooter shooterSubsystem;
    private final Intake intakeSubsystem;
    private final Vision visionSubsystem;

    private HolonomicDriveController controller;
    private final HolonomicPathFollowerConfig config;
    private final boolean isBlue;


    public AutoCommands(Swerve s_Swerve, Arm s_Arm, Indexer s_Indexer, Shooter s_Shooter, Intake s_Intake, Vision vision, Alliance alliance) {
        swerveSubsystem = s_Swerve;
        armSubsystem = s_Arm;
        indexerSubsystem = s_Indexer;
        shooterSubsystem = s_Shooter;
        intakeSubsystem = s_Intake;
        visionSubsystem = vision;

        if (alliance == Alliance.Blue) {
            this.isBlue = true;
        } else {
            this.isBlue =false;
        }
        this.config = new HolonomicPathFollowerConfig(
                new PIDConstants(100, 1 ,0),
                new PIDConstants(100, 1,0),
                krakenTalonConstants.Swerve.maxSpeed,
                krakenTalonConstants.Swerve.swerveRadius,
                new ReplanningConfig(true, false)
        );
    }

    public Command getSwerveMoveCommand(double x, double y, double rot) {
        return new AutoSwerve(swerveSubsystem, x, y, rot, true);
    }

    public Command getPrepareShootingCommand() {
        return new PrepareShootCommandGroup(armSubsystem, indexerSubsystem, intakeSubsystem, shooterSubsystem);
    }

    public Command getShootCommand() {
        return new ParallelDeadlineGroup(getToShootPosCommand().withTimeout(1.3), getPrepareShootingCommand());
    }

//    public Command getIntakingCommand() {
//        return new IntakeCommandGroup(indexerSubsystem, intakeSubsystem, shooterSubsystem);
//    }

//    public Command getIntakeCommand() {
//        return new ParallelDeadlineGroup(getIntakingCommand(), getToNoteCommand());
//    }

    public Command getTrajectoryFollowerCommand(String chosenPath) {
        return followPath(chosenPath);
    }

    public FollowPathHolonomic followPath(String chosenPath) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(chosenPath);
        return new FollowPathHolonomic(
                path,
                swerveSubsystem::getPose,
                swerveSubsystem::getLatestSpeeds,
                (ChassisSpeeds chassisSpeeds) -> swerveSubsystem.driveChassisSpeeds(chassisSpeeds, true),
                this.config,
                () -> this.isBlue,
                swerveSubsystem
        );
    }

    public Command getToShootPosCommand() {
        return new AutoSwerve(swerveSubsystem, 0, visionSubsystem.getAutoApproachPower(), visionSubsystem.getAngleToShootAngle(), false);
    }

     public Command getToNoteCommand() {
         return new AutoSwerve(swerveSubsystem, 0, 1, visionSubsystem.getNoteAimRotationPower(), false);
     }
}

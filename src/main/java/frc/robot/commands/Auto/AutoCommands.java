package frc.robot.commands.Auto;

import frc.robot.subsystems.Swerve;

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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;


public class AutoCommands {
    private final Swerve swerveSubsystem;
    private final Arm armSubsystem;
    private final Indexer indexerSubsystem;
    private final Shooter shooterSubsystem;
    private final Intake intakeSubsystem;

    private HolonomicDriveController controller;
    private final HolonomicPathFollowerConfig config;
    private final boolean isBlue;


    public AutoCommands(Swerve s_Swerve, Arm s_Arm, Indexer s_Indexer, Shooter s_Shooter, Intake s_Intake, Alliance alliance) {
        swerveSubsystem = s_Swerve;
        armSubsystem = s_Arm;
        indexerSubsystem = s_Indexer;
        shooterSubsystem = s_Shooter;
        intakeSubsystem = s_Intake;

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

    public Command getRevShooterCommand() {
        return new RevShooter(shooterSubsystem);
    }

    public Command getIntakeCommand() {
        return new IntakeCommandGroup(armSubsystem, indexerSubsystem, intakeSubsystem, shooterSubsystem);
    }

    public Command getTrajectoryFollowerCommand() {
        return new Command() {
            
        };
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

    // public Command getAlignToSpeakerCommand() {
    //     return new 
    // }

    // public Command getAlignToNoteCommand() {
    //     return new 
    // }
}

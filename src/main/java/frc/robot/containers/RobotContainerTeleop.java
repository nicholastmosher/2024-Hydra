package frc.robot.containers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Constants;
import frc.lib.Constants.AutonomousOptions;
import frc.lib.config.RobotConfig;
import frc.robot.classes.Limelight.LimelightHelpers;
import frc.robot.classes.ColorSensorController;
import frc.robot.classes.Limelight.LimelightController;
import frc.robot.commands.Auto.AutoCommands;
import frc.robot.commands.CommandGroups.IntakeCommands.IntakeCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.IntakeNoteCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.SendBackCommandGroup;
import frc.robot.commands.CommandGroups.ShootCommands.PrepareShootCommandGroup;
import frc.robot.commands.Drive.TeleopSwerve;

import frc.robot.commands.Indexer.FeedNote;
import frc.robot.commands.Intake.RejectNoteIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Light;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RobotContainerTeleop {
    /* Controllers */
    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    /* Subsystems */
    private final Swerve SwerveSubsystem;
    private final Arm ArmSubsystem;
    private final Intake IntakeSubsystem;
    private final Shooter ShooterSubsystem;
    private final Climber ClimberSubsystem;
    private final Indexer IndexerSubsystem;
    private final Light LightSubsystem;

    /* Util Classes */
    private final ColorSensorController colorSensorController;
    private final LimelightController limelightShootController;
    private final LimelightController limelightIntakeController;
    //private final AutoCommands autoCommandsConstructor;

    /* Teleop Commands */
    private final IntakeCommandGroup intakeCommand;
    private final PrepareShootCommandGroup prepareShootCommand;
    private final FeedNote feedNoteCommand;
    private final SendBackCommandGroup manualFeedBackCommand;
    private final RejectNoteIntake rejectNoteIntakeCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainerTeleop(RobotConfig robotConfig) {

        /* Util Classes */
        colorSensorController = new ColorSensorController(Constants.colorSensorConfig);
        limelightShootController = new LimelightController("limelight-shoot");

        limelightIntakeController = new LimelightController("limelight-intake");

        /* Subsystems */
        SwerveSubsystem = new Swerve(robotConfig.ctreConfigs);
        ArmSubsystem = new Arm(Constants.armConfig, robotConfig.dashboardConfig);
        IntakeSubsystem = new Intake(Constants.intakeConfig, colorSensorController);
        ShooterSubsystem = new Shooter(Constants.shooterConfig);
        IndexerSubsystem = new Indexer(Constants.indexerConfig);
        ClimberSubsystem = new Climber(Constants.climberConfig, robotConfig.dashboardConfig);
        LightSubsystem = new Light(Constants.lightConfig, colorSensorController);

        /* Teleop Commands */
        intakeCommand = new IntakeCommandGroup(ArmSubsystem, IndexerSubsystem, IntakeSubsystem, ShooterSubsystem);
        prepareShootCommand = new PrepareShootCommandGroup(ArmSubsystem, IndexerSubsystem, IntakeSubsystem, ShooterSubsystem);
        feedNoteCommand = new FeedNote(IndexerSubsystem);
        manualFeedBackCommand = new SendBackCommandGroup(IndexerSubsystem, ShooterSubsystem);
        rejectNoteIntakeCommand = new RejectNoteIntake(IntakeSubsystem);

        /* Command Constructor for Autos */
        //autoCommandsConstructor = new AutoCommands(SwerveSubsystem, ArmSubsystem, IndexerSubsystem, ShooterSubsystem, IntakeSubsystem, DriverStation.getAlliance().get());

        SwerveSubsystem.setDefaultCommand(
                new TeleopSwerve(
                        SwerveSubsystem,
                        () -> -pilot.getLeftX(),
                        () -> -pilot.getLeftY(),
                        () -> -pilot.getRightX(),
                        pilot.leftBumper(),
                        pilot.rightTrigger(),
                        pilot.leftTrigger(),
                        limelightShootController,
                        limelightIntakeController
                )
        );

        // ClimberSubsystem.setDefaultCommand(
        //         new InstantCommand(() -> ClimberSubsystem.joystickControl(copilot.getLeftY(), copilot.getRightY()), ClimberSubsystem)
        // );

        // LightSubsystem.setDefaultCommand(
        //         new InstantCommand(LightSubsystem::lightControl, LightSubsystem)
        // );

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* pilot Buttons */
        //pilot.leftTrigger().onTrue(intakeCommand);
        pilot.rightTrigger().whileTrue(prepareShootCommand);
        pilot.rightBumper().onTrue(feedNoteCommand.withTimeout(1));
        pilot.a().whileTrue(rejectNoteIntakeCommand);
        pilot.y().onTrue(new InstantCommand(SwerveSubsystem::zeroHeading));

        /* Copilot Buttons */
        copilot.rightBumper().onTrue(manualFeedBackCommand.withTimeout(0.7));
        
    }

    public Command getAutonomousCommand(AutonomousOptions plan) {
        // switch (plan) {
        //     case TWO_NOTE_CENTER:
        //         return 
        //     case SHOOT_NOTE:
        //         return 
        //     case SHOOT_NOTE_MOVEBACK:
        //         return 
        //     case RIGHTSPEAKERSIDESHOOTANDMOVEBACK:
        //         return 
        //     case THREE_NOTES_RIGHT:
        //         return 
        //     case THREE_NOTES_LEFT:
        //         return 
        //     default:
        //         return 
        // }
        return new InstantCommand();
    }
}

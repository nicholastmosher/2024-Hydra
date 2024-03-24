package frc.robot.containers;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Constants;
import frc.lib.Constants.AutonomousOptions;
import frc.lib.config.RobotConfig;
import frc.lib.config.krakenTalonConstants;
import frc.robot.classes.Limelight.LimelightHelpers;
import frc.robot.classes.ColorSensorController;
import frc.robot.classes.Limelight.LimelightController;
import frc.robot.commands.Arm.AmpPosition;
import frc.robot.commands.Auto.AutoCommands;
import frc.robot.commands.CPX.CpxSet;
import frc.robot.commands.CommandGroups.IntakeCommands.IntakeCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.IntakeNoteCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.SendBackCommandGroup;
import frc.robot.commands.CommandGroups.ShootCommands.PrepareShootCommandGroup;
import frc.robot.commands.Drive.HybridSwerve;
import frc.robot.commands.Drive.TeleopSwerve;

import frc.robot.commands.Indexer.FeedNote;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Intake.RejectNoteIntake;
import frc.robot.hybrid.BlendedSwerve;
import frc.robot.hybrid.HybridModes;
import frc.robot.hybrid.SwerveVector;
import frc.robot.subsystems.*;

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
    private final Vision VisionSubsystem;
    private final CPX CPXSubsystem;

    /* State Machine */
    private final RobotStateMachine robotStateMachine;

    /* Util Classes */
    private final ColorSensorController colorSensorController;
    //private final AutoCommands autoCommandsConstructor;

    /* Teleop Commands */
    private final IntakeCommandGroup intakeCommand;
    private final PrepareShootCommandGroup prepareShootCommand;
    private final FeedNote feedNoteCommand;
    private final SendBackCommandGroup manualFeedBackCommand;
    private final RejectNoteIntake rejectNoteIntakeCommand;
    private final CpxSet cpxOn;
    private final CpxSet cpxOff;

    private final AmpPosition ampPosition;


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainerTeleop(RobotConfig robotConfig) {

        /* Util Classes */
        colorSensorController = new ColorSensorController(Constants.colorSensorConfig);

        Pigeon2 gyro = new Pigeon2(krakenTalonConstants.Swerve.pigeonID);

        /* Subsystems */
        SwerveSubsystem = new Swerve(robotConfig.ctreConfigs, gyro);
        ArmSubsystem = new Arm(Constants.armConfig);
        IntakeSubsystem = new Intake(Constants.intakeConfig, colorSensorController);
        ShooterSubsystem = new Shooter(Constants.shooterConfig);
        IndexerSubsystem = new Indexer(Constants.indexerConfig);
        ClimberSubsystem = new Climber(Constants.climberConfig, robotConfig.dashboardConfig);
        LightSubsystem = new Light(Constants.lightConfig, colorSensorController);
        VisionSubsystem = new Vision(Constants.visionConfig);
        CPXSubsystem = new CPX(3); // TODO create CpxConfig

        /* State Machine */
        robotStateMachine = new RobotStateMachine();

        /* Teleop Commands */
        intakeCommand = new IntakeCommandGroup(IndexerSubsystem, IntakeSubsystem, ShooterSubsystem);
        prepareShootCommand = new PrepareShootCommandGroup(ArmSubsystem, IndexerSubsystem, IntakeSubsystem, ShooterSubsystem);
        feedNoteCommand = new FeedNote(IndexerSubsystem);
        manualFeedBackCommand = new SendBackCommandGroup(IndexerSubsystem, ShooterSubsystem);
        rejectNoteIntakeCommand = new RejectNoteIntake(IntakeSubsystem);
        cpxOn = new CpxSet(CPXSubsystem, true);
        cpxOff = new CpxSet(CPXSubsystem, false);

        ampPosition = new AmpPosition(ArmSubsystem);

        /* Command Constructor for Autos */
        //autoCommandsConstructor = new AutoCommands(SwerveSubsystem, ArmSubsystem, IndexerSubsystem, ShooterSubsystem, IntakeSubsystem, DriverStation.getAlliance().get());

        HybridModes modes = new HybridModes();
        modes.addMode("DriverControl", new SwerveVector(1.0, 1.0, 1.0));
        modes.addMode("DriverControl2", new SwerveVector(1.0, 1.0, 0.5));

        modes.addMode("IntakeCameraControl", new SwerveVector(0.0, 0.0, 0.0));
        modes.addMode("IntakeCameraControl2", new SwerveVector(0.0, 0.0, 1.0));

        modes.addMode("ShootCameraControl", new SwerveVector(0.0, 0.0, 0.0));
        modes.addMode("ShootCameraControl2", new SwerveVector(0.0, 0.0, 1.0));

        modes.addMode("OrientationControl", new SwerveVector(0,0,1));

        BlendedSwerve blendedSwerve = new BlendedSwerve();
        blendedSwerve.addComponent(
                () -> {
                    double x = MathUtil.applyDeadband(-pilot.getLeftX(), 0.1) * 4.5;
                    double y = MathUtil.applyDeadband(-pilot.getLeftY(), 0.1) * 4.5;
                    double rot = MathUtil.applyDeadband(-pilot.getRightX(), 0.1) * 5;
                    return new SwerveVector(x, y, rot);
                },
                () -> {
                    double t = MathUtil.applyDeadband(pilot.getLeftTriggerAxis(), 0.1);
                    SwerveVector control = modes.interpolate("DriverControl", "DriverControl2", t);
                    return control;
                }
        );
        blendedSwerve.addComponent(
                () -> new SwerveVector(0, 0, VisionSubsystem.getAngleToNote()),
                () -> {
                    double t = MathUtil.applyDeadband(pilot.getLeftTriggerAxis(), 0.1);
                    SwerveVector control = modes.interpolate("IntakeCameraControl", "IntakeCameraControl2", t);
                    return control;
                }
        );
        blendedSwerve.addComponent(
                () -> new SwerveVector(0, 0, VisionSubsystem.getAngleToShootAngle()),
                () -> {
                    double t = MathUtil.applyDeadband(pilot.getRightTriggerAxis(), 0.1);
                    SwerveVector control = modes.interpolate("ShootCameraControl", "ShootCameraControl2", t);
                    return control;
                }
        );
        PIDController gyroController = new PIDController(.1,0,0);
        blendedSwerve.addComponent(
                () -> {
                    double rotation = gyroController.calculate(gyro.getRotation2d().getDegrees(),0);;
                    return new SwerveVector(0,0,rotation);
                },
                () -> {
                    boolean active = pilot.povDown().getAsBoolean();
                    if (active){
                        // Full rotation control from gyro when D-Down
                        return new SwerveVector(0, 0, 1);
                    } else {
                        // No rotation control from gyro when no D-Down
                        return new SwerveVector(0, 0, 0);
                    }
                }
        );

        HybridSwerve hybridSwerve = new HybridSwerve(SwerveSubsystem, VisionSubsystem, blendedSwerve, pilot.leftBumper());
        SwerveSubsystem.setDefaultCommand(hybridSwerve);

//        SwerveSubsystem.setDefaultCommand(
//                new TeleopSwerve(
//                        SwerveSubsystem,
//                        VisionSubsystem,
//                        () -> -pilot.getLeftX(),
//                        () -> -pilot.getLeftY(),
//                        () -> -pilot.getRightX(),
//                        pilot.leftBumper(),
//                        pilot.rightTrigger(),
//                        pilot.leftTrigger()
//                )
//        );

//         ClimberSubsystem.setDefaultCommand(
//                 new InstantCommand(() -> ClimberSubsystem.joystickControl(copilot.getLeftY(), copilot.getRightY()), ClimberSubsystem)
//         );

         ArmSubsystem.setDefaultCommand(new InstantCommand(() -> ArmSubsystem.moveArm(MathUtil.applyDeadband(copilot.getLeftY(), 0.1)), ArmSubsystem));

//         LightSubsystem.setDefaultCommand(
//                 new InstantCommand(LightSubsystem::lightControl, LightSubsystem)
//         );

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* pilot Buttons */
        pilot.leftTrigger().onTrue(intakeCommand);
//        pilot.leftTrigger().onTrue(new InstantCommand(robotStateMachine::toggleIntaking));
        pilot.rightTrigger().whileTrue(prepareShootCommand);
        pilot.rightBumper().onTrue(feedNoteCommand.withTimeout(1));
        pilot.a().whileTrue(rejectNoteIntakeCommand);
        pilot.y().onTrue(new InstantCommand(SwerveSubsystem::zeroHeading));

        /* Copilot Buttons */
        copilot.rightBumper().onTrue(manualFeedBackCommand.withTimeout(0.7));
        copilot.x().onTrue(cpxOn);
        copilot.b().onTrue(cpxOff);

        copilot.a().whileTrue(ampPosition);
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


    public void robotPeriodic() {
        VisionSubsystem.periodic();
    }
}

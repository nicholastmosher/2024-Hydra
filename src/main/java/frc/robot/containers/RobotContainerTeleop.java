package frc.robot.containers;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Constants;
import frc.lib.Constants.AutonomousOptions;
import frc.lib.config.RobotConfig;
import frc.lib.config.krakenTalonConstants;
import frc.robot.classes.ColorSensorController;
import frc.robot.classes.ToggleHandler;
import frc.robot.commands.Arm.AmpPosition;
import frc.robot.commands.Auto.RevAuto;
import frc.robot.commands.Auto.ShootAuto;
import frc.robot.commands.CPX.CpxSet;
import frc.robot.commands.CommandGroups.IntakeCommands.IntakeCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.IntakeNoteCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.SendBackCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.ShuffleNote;
import frc.robot.commands.CommandGroups.ShootCommands.PrepareShootCommandGroup;
import frc.robot.commands.Drive.AutoSwerve;
import frc.robot.commands.Drive.HybridSwerve;

import frc.robot.commands.Indexer.FeedNote;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Intake.RejectNoteIntake;
import frc.robot.hybrid.BlendedControl;
import frc.robot.hybrid.HybridModes;
import frc.robot.hybrid.ControlVector;
import frc.robot.subsystems.*;

import java.sql.Driver;
import java.util.function.DoubleSupplier;

public class RobotContainerTeleop {
    /* Controllers */
    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    /* Instatiantion of pilot Triggers */

    private final int LeftYAxis = XboxController.Axis.kLeftY.value;
    private final int LeftXAxis = XboxController.Axis.kLeftX.value;
    private final int RightYAxis = XboxController.Axis.kRightY.value;
    private final int RightXAxis = XboxController.Axis.kRightX.value;

    private final int RightTriggerAxis = XboxController.Axis.kRightTrigger.value;
    private final int LeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;

    private final Trigger pilotRightTrigger = pilot.rightTrigger();
    private final Trigger pilotLeftTrigger = pilot.leftTrigger();
    private final Trigger copilotRightTrigger = copilot.rightTrigger();
    private final Trigger copilotLeftTrigger = copilot.leftTrigger();

    private final Trigger pilotRightBumper = pilot.rightBumper();
    private final Trigger copilotRightBumper = copilot.rightBumper();
    private final Trigger copilotLeftBumper = copilot.leftBumper();

    private final Trigger pilotyButton = pilot.y();
    private final Trigger pilotaButton = pilot.a();
    private final Trigger copilotaButton = copilot.a();

    private final Trigger copilotPOVup = copilot.povUp();
    private final Trigger copilotPOVleft = copilot.povLeft();
    private final Trigger copilotPOVright = copilot.povRight();
    private final Trigger copilotPOVdown = copilot.povDown();

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
    private final ToggleHandler shootAimOverideToggle;
    private final ToggleHandler intakeAimOverideToggle;
    private final ToggleHandler shortRangeOverrideToggle;
    private final ToggleHandler longRangeOverrideToggle;
    
    //private final AutoCommands autoCommandsConstructor;

    /* Teleop Commands */
    private final IntakeCommandGroup intakeCommand;
    private final PrepareShootCommandGroup prepareShootCommand;
    private final FeedNote feedNoteCommand;
    private final SendBackCommandGroup manualFeedBackCommand;
    private final RejectNoteIntake rejectNoteIntakeCommand;
    private final CpxSet cpxOn;
    private final CpxSet cpxOff;

    private final ShuffleNote shuffleNote;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainerTeleop(RobotConfig robotConfig) {

        /* Util Classes */
        colorSensorController = new ColorSensorController(Constants.colorSensorConfig);
        shootAimOverideToggle = new ToggleHandler();
        intakeAimOverideToggle = new ToggleHandler();
        shortRangeOverrideToggle = new ToggleHandler();
        longRangeOverrideToggle = new ToggleHandler();
        Pigeon2 gyro = new Pigeon2(krakenTalonConstants.Swerve.pigeonID);

        DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }

        /* Subsystems */
        SwerveSubsystem = new Swerve(robotConfig.ctreConfigs, gyro);
        ArmSubsystem = new Arm(Constants.armConfig);
        IntakeSubsystem = new Intake(Constants.intakeConfig, colorSensorController);
        ShooterSubsystem = new Shooter(Constants.shooterConfig);
        IndexerSubsystem = new Indexer(Constants.indexerConfig);
        ClimberSubsystem = new Climber(Constants.climberConfig, robotConfig.dashboardConfig);
        LightSubsystem = new Light(Constants.lightConfig, colorSensorController);
        VisionSubsystem = new Vision(Constants.visionConfig, alliance);
        CPXSubsystem = new CPX(3); // TODO create CpxConfig

        /* State Machine */
        robotStateMachine = new RobotStateMachine();

        /* Teleop Commands */
        intakeCommand = new IntakeCommandGroup(IndexerSubsystem, IntakeSubsystem, ShooterSubsystem, LightSubsystem);
        prepareShootCommand = new PrepareShootCommandGroup(ArmSubsystem, IndexerSubsystem, IntakeSubsystem, ShooterSubsystem);
        feedNoteCommand = new FeedNote(IndexerSubsystem);
        manualFeedBackCommand = new SendBackCommandGroup(IndexerSubsystem, ShooterSubsystem);
        rejectNoteIntakeCommand = new RejectNoteIntake(IntakeSubsystem);
        cpxOn = new CpxSet(CPXSubsystem, true);
        cpxOff = new CpxSet(CPXSubsystem, false);
        shuffleNote = new ShuffleNote(IndexerSubsystem, ShooterSubsystem);

        /* Command Constructor for Autos */
        //autoCommandsConstructor = new AutoCommands(SwerveSubsystem, ArmSubsystem, IndexerSubsystem, ShooterSubsystem, IntakeSubsystem, DriverStation.getAlliance().get());

        // Mode names in variables to prevent typos
        String modeDriverActive = "DriverControlActive";
        String modeDriverInactive = "DriverControlInactive";
        String modeIntakeAimInactive = "IntakeAimInactive";
        String modeIntakeAimActive = "IntakeAimActive";
        String modeShootAimInactive = "ShootAimInactive";
        String modeShootAimActive = "ShootAimActive";
        String modeShootDistanceInactive = "ShootDistanceInactive";
        String modeShootDistanceActive = "ShootDistanceActive";


        // Each mode describes an amount of influence that may be applied to each control
        HybridModes modes = new HybridModes();
        modes.addMode(modeDriverActive, ControlVector.fromFieldRelative(1.0, 1.0, 1.0).setSwerveRobotX(0.5).setSwerveRobotY(0.5));
        modes.addMode(modeDriverInactive, ControlVector.fromFieldRelative(1.0, 0.5, 0.5).setSwerveRobotX(0.5).setSwerveRobotY(0.5));
        modes.addMode(modeIntakeAimInactive, ControlVector.fromFieldRelative(0.0, 0.0, 0.0));
        modes.addMode(modeIntakeAimActive, ControlVector.fromFieldRelative(0.0, 0.0, 1.0));
        modes.addMode(modeShootAimInactive, ControlVector.fromFieldRelative(0.0, 0.0, 0.0));
        modes.addMode(modeShootAimActive, ControlVector.fromFieldRelative(0.0, 0.0, 1.0));
        // modes.addMode(modeShootDistanceInactive, ControlVector.fromRobotRelative(0, 0, 0));
        // modes.addMode(modeShootDistanceActive, ControlVector.fromRobotRelative(0,1, 0));


        // Each entry in the BlendedControl contributes some output to the Robot's movements
        BlendedControl blendedControl = new BlendedControl();
        blendedControl.addComponent(
                // Teleop Driver component
                () -> {
                    double fieldX = MathUtil.applyDeadband(-pilot.getRawAxis(LeftXAxis), 0.1) * 4.5;
                    double fieldY = MathUtil.applyDeadband(-pilot.getRawAxis(LeftYAxis), 0.1) * 4.5;
                    double robotX = MathUtil.applyDeadband(copilot.getRawAxis(LeftXAxis), 0.1) * 4.5;
                    double robotY = MathUtil.applyDeadband(copilot.getRawAxis(LeftYAxis), 0.1) * 4.5;


                    double rot = MathUtil.applyDeadband(-pilot.getRawAxis(RightXAxis), 0.1) * 5;
                    return ControlVector.fromFieldRelative(fieldX, fieldY, rot).setSwerveRobotX(robotX).setSwerveRobotY(robotY);
                },
                // TValue describes how much influence the Teleop Driver component has
                () -> {
                    double aimT = MathUtil.applyDeadband(pilot.getRawAxis(LeftTriggerAxis), 0.1);
                    ControlVector aimBlend = modes.interpolate(modeDriverActive, modeDriverInactive, aimT);

                    double armT = ArmSubsystem.percentRaised();
                    ControlVector armBlend = aimBlend.interpolate(ControlVector.fromFieldRelative(0.25,0.25,0.25), armT);
                    return armBlend;
                }
        );
        blendedControl.addComponent(
                () -> ControlVector.fromFieldRelative(0, 0, VisionSubsystem.getNoteAimRotationPower()),
                () -> {
                    double t = MathUtil.applyDeadband(pilot.getRawAxis(LeftTriggerAxis), 0.1);
                    if (intakeAimOverideToggle.get()) {
                        t=0;
                    }
                    return modes.interpolate(modeIntakeAimInactive, modeIntakeAimActive, t);
                }
        );
        blendedControl.addComponent(
                () -> ControlVector.fromFieldRelative(0, 0, VisionSubsystem.getAngleToShootAngle()),
                () -> {
                    double t = MathUtil.applyDeadband(pilot.getRawAxis(RightTriggerAxis), 0.1);;
                    if (shootAimOverideToggle.get()) {
                        t = 0;
                    }
                    return modes.interpolate(modeShootAimInactive, modeShootAimActive, t);
                }
        );

        // blendedControl.addComponent(
        //         () -> ControlVector.fromRobotRelative(0, VisionSubsystem.getAutoApproachPower(), 0),
        //         () -> {
        //             double t = MathUtil.applyDeadband(pilot.getRightTriggerAxis(), 0.2);
        //             if (shortRangeOverrideToggle.get()) {
        //                 t = 0;
        //             }
        //             ControlVector control = modes.interpolate(modeShootDistanceInactive, modeShootDistanceActive, t);
        //             return control;
        //         }
        // );
        PIDController gyroController = new PIDController(.1,0,0);
        blendedControl.addComponent(
                () -> {
                    double rotation = gyroController.calculate(gyro.getRotation2d().getDegrees(),0);;
                    return ControlVector.fromFieldRelative(0,0,rotation);
                },
                () -> {
                    boolean active = false;
                    if (active){
                        // Full rotation control from gyro when D-Down
                        return new ControlVector().setSwerveRotation(1);
                    } else {
                        // No rotation control from gyro when no D-Down
                        return new ControlVector().setSwerveRotation(0);
                    }
                }
        );

        // Long range auto-aiming: Lifting the arm
        blendedControl.addComponent(
                // PValue
                () -> {
//                    setpoint += copilot.getRawAxis(LeftYAxis);
                    ArmSubsystem.setTargetAngle(Rotation2d.fromRotations(Constants.armConfig.intakeAngle));
                    if (copilotLeftTrigger.getAsBoolean()) {
                        ArmSubsystem.setControlType(true);
                        ArmSubsystem.setTargetAngle(Rotation2d.fromRotations(Constants.armConfig.ampAngle));
                    }
                    if (pilotRightTrigger.getAsBoolean() && !copilotLeftTrigger.getAsBoolean()){
                        ArmSubsystem.setControlType(false);
                        ArmSubsystem.setTargetAngle(Rotation2d.fromRotations(Constants.armConfig.intakeAngle));
                    }
                    // Arm power from PID to target angle
                    double armPower = ArmSubsystem.getArmPowerToTarget();
                    return new ControlVector().setArmPower(armPower);
                },
                // TValue
                () -> {
                    
                        return new ControlVector().setArmPower(1);
                    
                }
        );

        HybridSwerve hybridSwerve = new HybridSwerve(SwerveSubsystem, blendedControl);
        SwerveSubsystem.setDefaultCommand(hybridSwerve);

        ClimberSubsystem.setDefaultCommand(
                new InstantCommand(() -> ClimberSubsystem.joystickControl(MathUtil.applyDeadband(copilot.getRawAxis(RightYAxis), 0.1)), ClimberSubsystem)
        );

        ArmSubsystem.setDefaultCommand(new InstantCommand(() -> {
            ControlVector control = blendedControl.solve();
            ArmSubsystem.moveArm(control.armPower());
//             ArmSubsystem.moveArm(MathUtil.applyDeadband(copilot.getRawAxis(LeftYAxis), 0.1));
        }, ArmSubsystem));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* pilot Buttons */
        pilotLeftTrigger.onTrue(intakeCommand);
        pilotRightTrigger.whileTrue(prepareShootCommand);
        pilotRightBumper.onTrue(new SequentialCommandGroup(feedNoteCommand.withTimeout(1), new InstantCommand(LightSubsystem::setWhite)));
        pilotaButton.whileTrue(rejectNoteIntakeCommand);
        pilotyButton.onTrue(new InstantCommand(SwerveSubsystem::zeroHeading));

        /* Copilot Buttons */
        copilotRightBumper.onTrue(manualFeedBackCommand.withTimeout(0.7));
        copilotPOVleft.onTrue(new InstantCommand(longRangeOverrideToggle::toggle));
        copilotPOVright.onTrue(new InstantCommand(shortRangeOverrideToggle::toggle));
        copilotPOVup.onTrue(new InstantCommand(shootAimOverideToggle::toggle));
        copilotPOVdown.onTrue(new InstantCommand(intakeAimOverideToggle::toggle));
        
        copilotaButton.onTrue(shuffleNote);
    }
    public Command getAutonomousCommand(AutonomousOptions plan) {
        switch (plan) {
            // case SHOOT_NOTE_MOVEBACK:
            //     return 
            // case RIGHTSPEAKERSIDESHOOTANDMOVEBACK:
            //     return 

            case SHOOT_NOTE:
                return shootNote();
            case TWO_NOTE_CENTER:
                return twoNoteCenterAuto();
            case THREE_NOTES_RIGHT:
                return threeNoteRightAuto();
            case THREE_NOTES_LEFT:
                return threeNoteLeftAuto();
            default:
                return shootNote();
        }
    }


    public void robotPeriodic() {
        VisionSubsystem.periodic();
    }

    public Command twoNoteCenterAuto() {
        return new SequentialCommandGroup(shootNote(),scoreCenterNote());
    }

    public Command threeNoteLeftAuto() {
        return new SequentialCommandGroup(shootNote(),scoreCenterNote(), scoreLeftNote());
    }

    public Command threeNoteRightAuto() {
        return new SequentialCommandGroup(shootNote(),scoreCenterNote(), scoreRightNote());
    }

    public Command fourNoteAuto() {
        return new SequentialCommandGroup(shootNote(),scoreCenterNote(), scoreLeftNote(), scoreRightNote());
    }

    public Command shootNote (){
        return new SequentialCommandGroup(
            new RevAuto(ShooterSubsystem).withTimeout(1),
            new ShootAuto(ShooterSubsystem, IndexerSubsystem).withTimeout(0.5)
        );
    }

    public Command scoreCenterNote() {
        Command backupAndIntake = new ParallelDeadlineGroup(
            backupToCenterNote(),
            new IntakeNoteCommandGroup(IntakeSubsystem, IndexerSubsystem).withTimeout(2)
        );

        Command returnToSpeakerAndShuffle = new ParallelDeadlineGroup(
            new ShuffleNote(IndexerSubsystem, ShooterSubsystem),
            centerNoteReturnToSpeaker()
        );

        return new SequentialCommandGroup(
            backupAndIntake.withTimeout(3),
            returnToSpeakerAndShuffle.withTimeout(3),
            shootNote()
        );
    }

    public Command scoreLeftNote() {
        Command backupAndIntake = new ParallelDeadlineGroup(
            backupToLeftNote(),
            new IntakeNoteCommandGroup(IntakeSubsystem, IndexerSubsystem).withTimeout(2)
        );

        Command returnToSpeakerAndShuffle = new ParallelDeadlineGroup(
            new ShuffleNote(IndexerSubsystem, ShooterSubsystem),
            leftNoteReturnToSpeaker()
        );

        return new SequentialCommandGroup(
            backupAndIntake.withTimeout(3),
            returnToSpeakerAndShuffle.withTimeout(3),
            shootNote()
        );
    }

    public Command scoreRightNote() {
        Command backupAndIntake = new ParallelDeadlineGroup(
            backupToRightNote(),
            new IntakeNoteCommandGroup(IntakeSubsystem, IndexerSubsystem).withTimeout(2)
        );

        Command returnToSpeakerAndShuffle = new ParallelDeadlineGroup(
            new ShuffleNote(IndexerSubsystem, ShooterSubsystem),
            rightNoteReturnToSpeaker()
        );

        return new SequentialCommandGroup(
            backupAndIntake.withTimeout(3),
            returnToSpeakerAndShuffle.withTimeout(3),
            shootNote()
        );
    }

    public Command backupToCenterNote() {
        return new AutoSwerve(SwerveSubsystem, 0, -0.3, 0,false).withTimeout(2);
    }

    public Command centerNoteReturnToSpeaker() {
        return new AutoSwerve(SwerveSubsystem, 0, 0.3, 0,false).withTimeout(2);
    }

    public Command backupToLeftNote() {
        return new AutoSwerve(SwerveSubsystem, -0.4, -0.3, 0,false).withTimeout(1);
    }

    public Command leftNoteReturnToSpeaker() {
        return new AutoSwerve(SwerveSubsystem, 0.4, 0.3, 0,false).withTimeout(1);
    }

    public Command backupToRightNote() {
        return new AutoSwerve(SwerveSubsystem, 0.4, -0.3, 0,false).withTimeout(1);
    }

    public Command rightNoteReturnToSpeaker() {
        return new AutoSwerve(SwerveSubsystem, -0.4, 0.3, 0,false).withTimeout(1);
    }
}

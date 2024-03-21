package frc.robot.containers;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.lib.Constants;
import frc.lib.Constants.AutonomousOptions;
import frc.lib.config.RobotConfig;
import frc.robot.Robot;
import frc.robot.classes.BlinkinLEDController;
import frc.robot.commands.Drive.AutoSwerve;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Autos.RevAuto;
import frc.robot.commands.Autos.ShootAuto;
import frc.robot.commands.Autos.StopShooterAuto;
import frc.robot.commands.Autos.TrajectoryFollowerCommands;
import frc.robot.commands.Shooter.FeedNote;
import frc.robot.commands.Shooter.RevShooter;
import frc.robot.commands.Vision.limeLightOff;
import frc.robot.commands.Vision.limeLightOn;
import frc.robot.commands.Intake.RejectNoteIntake;
import frc.robot.commands.Light.SetRed;
import frc.robot.commands.Light.SetWhite;
import frc.robot.commands.Shooter.*;
import frc.robot.interfaces.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.CommandGroups.Intake.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainerTeleop implements RobotContainer {
    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController teloscopicControl = new CommandXboxController(1);

    /* Drive Controls */
    private final int leftyAxis = XboxController.Axis.kLeftY.value;
    private final int leftxAxis = XboxController.Axis.kLeftX.value;
    private final int rightyAxis = XboxController.Axis.kRightY.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */

    /* Subsystems */
    private final Swerve s_Swerve;
    private final Arm a_Arm;
    private final Intake i_Intake;
    private final Shooter s_Shooter;
    private final Climber c_Climber;
    // private final Vision v_Vision;

    private final TrajectoryFollowerCommands pathFollower;
    private final FeedNote feedNote;
    private final IntakingCommandGroup intaking;
    private final RevShooter revShooter;
    // private final limeLightOff lightOff;
    // private final limeLightOn lightOn;
    private final RejectNoteIntake rejectNoteIntake;
    private final SendBack sendBack;
    private final StopIntake stopIntake;
    private final StopShooter stopShooter;
    private final SetRed setRed;
    private final SetWhite setWhite;
    private final SendBackSecond sendBackShooter;

    private final IntakingCommandGroup intakingAuto;
    private final RevAuto revAuto;
    private final RevAuto revAuto2;
    private final ShootAuto shootAuto;
    private final ShootAuto shootAuto2;
    private final StopShooterAuto stopShooterAuto;
    private final StopShooterAuto stopShooterAuto2;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainerTeleop(RobotConfig robotConfig, BlinkinLEDController blinkin) {
        this.s_Swerve = new Swerve(robotConfig.ctreConfigs);
        this.pathFollower = new TrajectoryFollowerCommands(s_Swerve, true);


        a_Arm = new Arm(Constants.armConfig, robotConfig.dashboardConfig);
        i_Intake = new Intake(Constants.intakeConfig);
        s_Shooter = new Shooter(Constants.shooterConfig);
        c_Climber = new Climber(Constants.climberConfig, robotConfig.dashboardConfig);

        // Commands
        feedNote = new FeedNote(s_Shooter, i_Intake);
        intaking = new IntakingCommandGroup(i_Intake, s_Shooter);
        revShooter = new RevShooter(s_Shooter);
        sendBack = new SendBack(s_Shooter);
        rejectNoteIntake = new RejectNoteIntake(i_Intake);
        stopIntake = new StopIntake(s_Shooter, i_Intake);
        stopShooter = new StopShooter(s_Shooter);
        setRed = new SetRed(blinkin);
        setWhite = new SetWhite(blinkin);
        sendBackShooter = new SendBackSecond(s_Shooter);

        revAuto = new RevAuto(s_Shooter);
        revAuto2 = new RevAuto(s_Shooter);
        shootAuto = new ShootAuto(s_Shooter);
        shootAuto2 = new ShootAuto(s_Shooter);
        intakingAuto = new IntakingCommandGroup(i_Intake, s_Shooter);
        stopShooterAuto = new StopShooterAuto(s_Shooter);
        stopShooterAuto2 = new StopShooterAuto(s_Shooter);



        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve,
                () -> -driver.getRawAxis(leftxAxis),
                () -> -driver.getRawAxis(leftyAxis),
                () -> -driver.getRawAxis(rotationAxis),
                    driver.leftBumper()
            )
        );

//        c_Climber.setDefaultCommand(
//                new InstantCommand(() -> c_Climber.joystickControl(teloscopicControl.getRawAxis(leftyAxis), teloscopicControl.getRawAxis(rightyAxis)), c_Climber)
//        );
        a_Arm.setDefaultCommand(
                new InstantCommand(() -> a_Arm.moveArm(teloscopicControl.getRawAxis(rightyAxis)), a_Arm)
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.y().onTrue(new InstantCommand(s_Swerve::zeroHeading));
        driver.leftTrigger().onTrue(new SequentialCommandGroup(intaking, setRed, sendBack.withTimeout(0.7), stopIntake));//.onFalse(new SequentialCommandGroup(sendBack.withTimeout(1), stopIntake));
        driver.rightTrigger().whileTrue(revShooter);//onTrue(revShooter.onlyIf(s_Shooter::isShooterStopped));//toggleOnTrue(new SequentialCommandGroup(revShooter.onlyIf()stopShooter.onlyIf(s_Shooter::isShooterStopped)));//whileTrue(revShooter).onFalse(stopShooter);//.toggleOnFalse(new InstantCommand(s_Shooter::stopShoot));
        driver.rightBumper().onTrue(new SequentialCommandGroup(feedNote.withTimeout(1), setWhite));
        driver.a().whileTrue(rejectNoteIntake);
      
        // teloscopicControl.x().onTrue(lightOn);
        // teloscopicControl.y().onTrue(lightOff);
        //teloscopicControl.rightBumper().whileTrue(sendBack);
        teloscopicControl.rightBumper().onTrue(sendBackShooter.withTimeout(0.7));



//        teloscopicControl.x().onTrue(lightOn);
//        teloscopicControl.y().onTrue(lightOff);
        //.onFalse(new InstantCommand(s_Shooter::stopFeed))
    }

    private Command revAuto() {
        return new RevAuto(s_Shooter);
    }

    private Command intakingAuto() {
        return new IntakingCommandGroup(i_Intake, s_Shooter);
    }

    private Command shootAuto() {
        return new ShootAuto(s_Shooter);
    }

    private Command stopShooterAuto() {
        return new StopShooterAuto(s_Shooter);
    }

    private Command swerveMoveRight() {
        return new AutoSwerve(s_Swerve, 0.3, 0, 0, true);
    }

    private Command swerveMoveLeft() {
        return new AutoSwerve(s_Swerve, 0.3, 0, 0, true);
    }

    private Command swerveMoveBack() {
        return new AutoSwerve(s_Swerve, 0, 0.3, 0, true);
    }

    private Command swerveBackRight() {
        return new AutoSwerve(s_Swerve, 0.3, 0.3, 0, true);
    }

    private Command swerveBackLeft() {
        return new AutoSwerve(s_Swerve, -0.3, 0.3, 0.1, true);
    }

    private Command swerveFrontLeft() {
        return new AutoSwerve(s_Swerve, -0.2875, -0.3, 0, true);
    }

    private Command swerveFrontRight() {
        return new AutoSwerve(s_Swerve, 0.3, -0.35, -0.1, true);
    }

    private Command swerveMoveForward() {
        return new AutoSwerve(s_Swerve, 0, -0.3, 0, true);
    }

    private Command moveBackAmpSide() {
        return new AutoSwerve(s_Swerve, -0.2, 0.25, 0, true).withTimeout(4);
    }

    private Command moveBackSource() {
        return new AutoSwerve(s_Swerve, -0.2, -0.25, 0, true).withTimeout(4);
    }

    private Command intakeCommand() {
        return new IntakingCommandGroup(i_Intake, s_Shooter);
    }

    private Command twoNoteCenterAuto() {
        Command SwerveMoveBack = swerveMoveBack();
        Command SwerveMoveForward = swerveMoveForward();
        Command firstShoot = shootNote();
        Command backPickup = new ParallelDeadlineGroup(intakingAuto(), SwerveMoveBack.withTimeout(2));
        Command forwardAndRev = new ParallelCommandGroup(revAuto(), SwerveMoveForward).withTimeout(2.3);
        Command shoot = new SequentialCommandGroup(shootAuto2.withTimeout(1), stopShooterAuto2.withTimeout(0.5));
        Command twoNoteAuto = new SequentialCommandGroup(firstShoot, backPickup, forwardAndRev, shoot);
        return twoNoteAuto;
    }

    private Command backRightPickup() {
        return new ParallelDeadlineGroup(
            intakeCommand(),
            swerveBackRight()
        );
    }

    private Command backLeftPickup() {
        return new ParallelDeadlineGroup(
            intakeCommand(),
            swerveBackLeft()
        );
    }

    private Command rightAuto() {
        return new SequentialCommandGroup(
            backRightPickup().withTimeout(2),
            swerveFrontLeft().withTimeout(2),
            shootNote()
        );
    }

    private Command threeNoteCenterRightAuto() {
        Command backRightPickup = new ParallelDeadlineGroup(
            intakingAuto(),
            swerveBackRight()
        );

        return new SequentialCommandGroup(
            twoNoteCenterAuto(),
            backRightPickup.withTimeout(2),
            swerveFrontLeft().withTimeout(2),
            shootNote()
        );
    }

    private Command threeNoteCenterLeftAuto() {
        Command intakingAuto = intakeCommand();
        Command backLeftPickup = new ParallelDeadlineGroup(
            intakingAuto(),
            swerveBackLeft()
        );

        return new SequentialCommandGroup(
            twoNoteCenterAuto(),
            backLeftPickup.withTimeout(2),
            swerveFrontRight().withTimeout(2),
            shootNote()
        );
    }

    private Command twoNoteCenterAutoWithBackup() {
        Command SwerveMoveBack = swerveMoveBack();
        Command SwerveMoveForward = swerveMoveForward();
        Command firstShoot = shootNote();
        Command backPickup = new ParallelDeadlineGroup(intakingAuto(), SwerveMoveBack.withTimeout(2));
        Command forwardAndRev = new ParallelCommandGroup(revAuto(), SwerveMoveForward).withTimeout(2.3);
        Command shoot = new SequentialCommandGroup(shootAuto2.withTimeout(1), stopShooterAuto2.withTimeout(0.5));
        //Command swerveMoveLeft = swerveMoveLeft();
        Command moveBackAmpSide = moveBackAmpSide();
        Command twoNoteAuto = new SequentialCommandGroup(firstShoot, backPickup, forwardAndRev, shoot, moveBackAmpSide);
        return twoNoteAuto;
    }

    private Command shootNote() {
        return new SequentialCommandGroup(
            revAuto().withTimeout(1.5), 
            shootAuto().withTimeout(1), 
            stopShooterAuto().withTimeout(0.5)
        );
    }

    private Command rightSpeakerSideShootandMoveBack() {
        return new SequentialCommandGroup(shootNote(), moveBackAmpSide());
    }

    // private Command centerShootWaitBackup() {
    //     Command wait = new InstantCommand(() -> {});
    //     return new SequentialCommandGroup(
    //         shootNote(),
    //         wait.withTimeout(10),
            
    //     );
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutonomousCommand(AutonomousOptions plan) {
        //return moveBackAmpSide();
        switch (plan) {
            case TWO_NOTE_CENTER:
                return twoNoteCenterAuto();
            case SHOOT_NOTE:
                return shootNote();
            case SHOOT_NOTE_MOVEBACK:
                return twoNoteCenterAutoWithBackup();
            case RIGHTSPEAKERSIDESHOOTANDMOVEBACK:
                return rightSpeakerSideShootandMoveBack();
            case THREE_NOTES_RIGHT:
                return threeNoteCenterRightAuto();
            case THREE_NOTES_LEFT:
                return threeNoteCenterLeftAuto();
            default:
                return shootNote();
        }

    }

    @Override
    public void robotPeriodic() {
    }

}

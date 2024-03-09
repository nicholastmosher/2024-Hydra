package frc.robot.containers;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.lib.Constants;
import frc.lib.config.RobotConfig;
import frc.robot.Robot;
import frc.robot.classes.BlinkinLEDController;
import frc.robot.commands.Drive.AutoSwerve;
import frc.robot.commands.Drive.DefensePos;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Autos.ShootAuto;
import frc.robot.commands.Autos.TrajectoryFollowerCommands;
import frc.robot.commands.Drive.ZeroHeading;
import frc.robot.commands.Initialize.ClimberInit;
import frc.robot.commands.Shooter.FeedNote;
import frc.robot.commands.Shooter.RevShooter;
import frc.robot.commands.Vision.limeLightOff;
import frc.robot.commands.Vision.limeLightOn;
import frc.robot.commands.Intake.RejectNoteIntake;
import frc.robot.commands.Light.SetRed;
import frc.robot.commands.Light.SetWhite;
import frc.robot.commands.Shooter.SendBackSecond;
import frc.robot.commands.Shooter.*;
import frc.robot.interfaces.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.CommandGroups.Intake.*;

import java.beans.FeatureDescriptor;

import static frc.lib.Constants.AutonomousOptions;
import static frc.robot.classes.BlinkinLEDController.BlinkinPattern.SHOT_RED;
//import frc.robot.commands.CommandGroups.Shoot.*;

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
    //private final Arm a_Arm;
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
    private final ShootAuto shootAuto;



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainerTeleop(RobotConfig robotConfig, BlinkinLEDController blinkin) {

        this.s_Swerve = new Swerve(robotConfig.ctreConfigs);
        this.pathFollower = new TrajectoryFollowerCommands(s_Swerve, true);

        //a_Arm = new Arm(Constants.armConfig, robotConfig.dashboardConfig);
        i_Intake = new Intake(Constants.intakeConfig);
        s_Shooter = new Shooter(Constants.shooterConfig);
        c_Climber = new Climber(Constants.climberConfig, robotConfig.dashboardConfig);
        //v_Vision = new  Vision();
        feedNote = new FeedNote(s_Shooter, i_Intake);
        intaking = new IntakingCommandGroup(i_Intake, s_Shooter);
        revShooter = new RevShooter(s_Shooter);
        // lightOff = new limeLightOff(v_Vision);
        // lightOn = new limeLightOn(v_Vision);
        
        

        sendBack = new SendBack(s_Shooter);
        rejectNoteIntake = new RejectNoteIntake(i_Intake);
        stopIntake = new StopIntake(s_Shooter, i_Intake);
        stopShooter = new StopShooter(s_Shooter);
        setRed = new SetRed(blinkin);
        setWhite = new SetWhite(blinkin);
        sendBackShooter = new SendBackSecond(s_Shooter);

        shootAuto = new ShootAuto(s_Shooter);
        intakingAuto = new IntakingCommandGroup(i_Intake, s_Shooter);

       s_Swerve.setDefaultCommand(
           new TeleopSwerve(
               s_Swerve,
               () -> -driver.getRawAxis(leftxAxis),
               () -> -driver.getRawAxis(leftyAxis),
               () -> -driver.getRawAxis(rotationAxis),
                   driver.leftBumper()
           )
       );

        c_Climber.setDefaultCommand(
                new InstantCommand(() -> c_Climber.joystickControl(teloscopicControl.getRawAxis(leftyAxis)), c_Climber)
        );
//        a_Arm.setDefaultCommand(
//                new InstantCommand(() -> a_Arm.moveArm(teloscopicControl.getRawAxis(rightyAxis)), a_Arm)
//        );

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
      
   



        //.onFalse(new InstantCommand(s_Shooter::stopFeed))
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutonomousCommand1() {
//        Command autoCommand = new TeleopSwerve(s_Swerve);
//
//        switch(sp) {
//            case DRIVE:
//                break;
//            case DRIVEMIDDLE:
//                autCommand = new AutoSequencePlaceCube(m_robotDrive, m_Arm, m_vision, m_grabberSubsystem, true);
//                break;
//            case SHOOTNDRIVE:
//                autCommand = new AutoSequencePlaceCube(m_robotDrive, m_Arm, m_vision, m_grabberSubsystem, false);
//                break;
//
//        }
//        return autoCommand;
        //return pathFollower.followPath("Line"); 
        return shootAuto.withTimeout(1.5);//new SequentialCommandGroup(shootAuto.withTimeout(0.3), new ParallelDeadlineGroup(intakingAuto, SwerveMoveBack));//.withTimeout(2));//SwerveMoveBack;//shootAuto.withTimeout(1.5);
        // return new SequentialCommandGroup(new ParallelDeadlineGroup(feedNote, revShooter), s_Swerve.getDefaultCommand());
    }

    
    @Override
    public Command getAutonomousCommand2() {
        Command SwerveMoveBack = new AutoSwerve(s_Swerve, 0, 0.3, 0, true);
        return new ParallelDeadlineGroup(intakingAuto, SwerveMoveBack);
    }

//    @Override
//    public Command Initialize() {
//        return climberInit;
//    }


//    public Command Initizalize() {
//        return climberInit;
//    }

    @Override
    public void robotPeriodic() {
        // SwerveModuleState[] swerveStates = s_Swerve.getModuleStates();
        // for (int i = 0; i < swerveStates.length; i++) {
        //     SwerveModuleState state = swerveStates[i];
        //     SmartDashboard.putNumber(String.format("SwerveSpeed%d", i), state.speedMetersPerSecond);
        // }
        // s_Shooter.dashboardPeriodic();

    }

}

package frc.robot.containers;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.lib.Constants;
import frc.lib.config.RobotConfig;
import frc.robot.Robot;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Autos.TrajectoryFollowerCommands;
import frc.robot.commands.Drive.ZeroHeading;
import frc.robot.commands.Initialize.ClimberInit;
import frc.robot.commands.Shooter.FeedNote;
import frc.robot.commands.Shooter.RevShooter;
import frc.robot.commands.Shooter.SendBack;
import frc.robot.interfaces.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.CommandGroups.Intake.*;

import static frc.lib.Constants.AutonomousOptions;
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
    private final Arm a_Arm;
    private final Intake i_Intake;
    private final Shooter s_Shooter;
    private final Climber c_Climber;
    //private final PowerDistribution p_Power;
    // command groups

    private final ZeroHeading zero;
    private final TrajectoryFollowerCommands pathFollower;

    private final FeedNote feedNote;
    private final IntakingCommandGroup intaking;
    private final RevShooter revShooter;
    private final SendBack sendBack;

    private final ClimberInit climberInit;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainerTeleop(RobotConfig robotConfig) {

        this.s_Swerve = new Swerve(robotConfig.ctreConfigs);
        this.pathFollower = new TrajectoryFollowerCommands(s_Swerve, true);

        a_Arm = new Arm(Constants.armConfig, robotConfig.dashboardConfig);
        i_Intake = new Intake(Constants.intakeConfig);
        s_Shooter = new Shooter(Constants.shooterConfig);
        c_Climber = new Climber(Constants.climberConfig, robotConfig.dashboardConfig);

        zero = new ZeroHeading(s_Swerve);

        feedNote = new FeedNote(s_Shooter, i_Intake);
        intaking = new IntakingCommandGroup(i_Intake, s_Shooter);
        revShooter = new RevShooter(s_Shooter);
        sendBack = new SendBack(s_Shooter, 1);

        climberInit = new ClimberInit(c_Climber);

//       s_Swerve.setDefaultCommand(
//           new TeleopSwerve(
//               s_Swerve,
//               () -> -driver.getRawAxis(leftxAxis),
//               () -> -driver.getRawAxis(leftyAxis),
//               () -> -driver.getRawAxis(rotationAxis),
//                   driver.b()
//           )
//       );

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
        driver.leftTrigger().whileTrue(intaking).onFalse(sendBack);
        driver.rightTrigger().whileTrue(revShooter);//.toggleOnFalse(new InstantCommand(s_Shooter::stopShoot));
        driver.rightBumper().whileTrue(feedNote);
        //.onFalse(new InstantCommand(s_Shooter::stopFeed))
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutonomousCommand() {
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
        return pathFollower.followPath("shortline");
    }

    @Override
    public Command Initialize() {
        return climberInit;
    }


    public Command Initizalize() {
        return climberInit;
    }

    @Override
    public void robotPeriodic() {
        SwerveModuleState[] swerveStates = s_Swerve.getModuleStates();
        for (int i = 0; i < swerveStates.length; i++) {
            SwerveModuleState state = swerveStates[i];
            SmartDashboard.putNumber(String.format("SwerveSpeed%d", i), state.speedMetersPerSecond);
        }
        s_Shooter.dashboardPeriodic();

    }
}

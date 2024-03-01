package frc.robot.containers;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.lib.Constants;
import frc.lib.config.RobotConfig;
import frc.robot.Robot;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Autos.TrajectoryFollowerCommands;
import frc.robot.interfaces.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.CommandGroups.Intake.*;
import frc.robot.commands.CommandGroups.Shoot.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainerTeleop implements RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick teloscopicControl = new Joystick(0);

    /* Drive Controls */
    private final int yAxis = XboxController.Axis.kLeftY.value;
    private final int xAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton intake = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton shoot = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);

    /* Subsystems */
    private final Swerve s_Swerve;
    private final Arm a_Arm;
    private final Intake i_Intake;
    private final Shooter s_Shooter;
    private final Climber c_Climber;
    private final PowerDistribution p_Power;
    // command groups

    private final IntakeCommandGroup intakeCommand;
    private final ShootCommandGroup shootCommand;

    private final TrajectoryFollowerCommands pathFollower;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainerTeleop(RobotConfig robotConfig) {
        this.s_Swerve = new Swerve(robotConfig.ctreConfigs);
        this.pathFollower = new TrajectoryFollowerCommands(s_Swerve, true);
        a_Arm = new Arm(Constants.armConfig, robotConfig.dashboardConfig);
        i_Intake = new Intake(Constants.intakeConfig);
        s_Shooter = new Shooter(Constants.shooterConfig);
        c_Climber = new Climber(Constants.climberConfig, robotConfig.dashboardConfig);
        p_Power = new PowerDistribution(0, PowerDistribution.ModuleType.kRev);

        intakeCommand = new IntakeCommandGroup(a_Arm, i_Intake, s_Shooter);
        shootCommand = new ShootCommandGroup(a_Arm, s_Shooter);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(xAxis),
                () -> -driver.getRawAxis(yAxis),
                () -> -driver.getRawAxis(rotationAxis),
                    robotCentric
            )
        );

        c_Climber.setDefaultCommand(
                new InstantCommand(() -> c_Climber.joystickControl(teloscopicControl.getRawAxis(yAxis)), c_Climber)
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
        zeroGyro.onTrue(new InstantCommand(s_Swerve::zeroHeading));
        //intake.whileTrue(intakeCommand);
        //shoot.whileTrue(shootCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutonomousCommand() {
        return pathFollower.followPath("shortline");
    }

    @Override
    public void robotPeriodic() {
        SwerveModuleState[] swerveStates = s_Swerve.getModuleStates();
        for (int i = 0; i < swerveStates.length; i++) {
            SwerveModuleState state = swerveStates[i];
            SmartDashboard.putNumber(String.format("SwerveSpeed%d", i), state.speedMetersPerSecond);
        }
        s_Shooter.dashboardPeriodic();
        SmartDashboard.putBoolean("robotcentric", !robotCentric.getAsBoolean());
    }
}

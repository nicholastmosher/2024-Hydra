package frc.robot.containers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.lib.Constants;
import frc.lib.CtreConfigs;
import frc.lib.config.ArmConfig;
import frc.lib.config.RobotConfig;
import frc.robot.Robot;
import frc.robot.commands.CommandGroups.Intake.IntakingCommandGroup;
import frc.robot.commands.Drive.debug.*;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Shooter.IndexNote;
import frc.robot.interfaces.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.commands.Arm.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainerTest implements RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    //private final Joystick driver2 = new Joystick(1);
    private final int yAxis = XboxController.Axis.kLeftY.value;
    private final JoystickButton amp = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton shoot = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton intake = new JoystickButton(driver,XboxController.Button.kY.value);
    private final JoystickButton debugSwerveRotation = new JoystickButton(driver,XboxController.Button.kX.value);
    private final JoystickButton Intaking = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final Swerve mSwerve;
    private final Arm a_Arm;
    private final Shooter s_Shooter = new Shooter(Constants.shooterConfig);
    private final Intake i_intake = new Intake(Constants.intakeConfig);


    private final IntakePosition armtoIntake;
    private final ShootPosition armtoShoot;
    private final AmpPosition armtoAmp;
    private final IntakingCommandGroup intaking = new IntakingCommandGroup(i_intake, s_Shooter);
    private final IntakeNote intakingg = new IntakeNote(i_intake);
    private final IndexNote index = new IndexNote(s_Shooter);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainerTest(RobotConfig robotConfig) {
        this.mSwerve = new Swerve(robotConfig.ctreConfigs);
        // Configure the button bindings
        configureButtonBindings();
        System.out.println("Initialized DebugRobot");
        a_Arm = new Arm(Constants.armConfig, robotConfig.dashboardConfig);
        a_Arm.setDefaultCommand(
                new InstantCommand(() -> a_Arm.moveArm(driver.getRawAxis(yAxis)), a_Arm)
        );

        this.armtoIntake = new IntakePosition(a_Arm);
        this.armtoShoot = new ShootPosition(a_Arm);
        this.armtoAmp = new AmpPosition(a_Arm);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        //debugSwerveTranslation.onTrue(new DiagnoseSwerveTranslation(mSwerve));
        //debugSwerveRotation.onTrue(new DiagnoseSwerveRotation(mSwerve));

        //commandDrive.onTrue(new DiagnoseDrive(mSwerve));
        //commandDrive.onTrue(new InstantCommand(() -> mSwerve.zeroEncoders()));
        //commandSteer.onTrue(new DiagnoseSteering(mSwerve));
//        commandSteer.onTrue(new SwerveAssignSteer(motorTest));
//        commandShoot.onTrue(new ShooterAssignPower(mShooter, 0.70));
        //amp.whileTrue(this.armtoAmp);
        //shoot.whileTrue(this.armtoShoot);
        //intake.whileTrue(this.armtoIntake);
        amp.whileTrue(intakingg);
        shoot.whileTrue(index);


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    @Override
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        return new InstantCommand(() -> {});
    }
}

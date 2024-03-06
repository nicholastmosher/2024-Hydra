package frc.robot.commands.CommandGroups.Intake;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.IntakePosition;
import frc.robot.commands.Shooter.SendBack;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeCommandGroup extends SequentialCommandGroup {
    public IntakeCommandGroup(Intake intake, Shooter shooter) {
        super(new IntakingCommandGroup(intake, shooter), new SendBack(shooter));
    }
}
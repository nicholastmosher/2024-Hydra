package frc.robot.commands.CommandGroups.Shoot;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.ShootPosition;
import frc.robot.commands.Shooter.ShootNote;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;

public class ShootCommandGroup extends SequentialCommandGroup {
    public ShootCommandGroup(Arm arm, Shooter shooter) {
        super(new ShootPosition(arm), new ShootNote(shooter));
    }
}
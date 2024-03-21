package frc.robot.commands.CPX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CPX;

public class CpxSet extends Command {
    /** Creates a new limeLightOff. */
    private CPX m_cpx;
    private boolean m_status;

    public CpxSet(CPX l, boolean status) {
        this.m_cpx = l;
        this.m_status = status;
        addRequirements(l);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_cpx.set(m_status);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
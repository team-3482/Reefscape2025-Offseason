package frc.robot.pivot;

import edu.wpi.first.wpilibj2.command.Command;

/** A command that pushes the pivot down to the hardstop to zero it */
public class ZeroPivotCommand extends Command {

    /** Creates a new ZeroPivotCommand. */
    public ZeroPivotCommand() {
        setName("ZeroPivotCommand");

        addRequirements(PivotSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        // PivotSubsystem.getInstance().setPivotSpeed(PivotConstants.ZERO_PIVOT_VOLTAGE / 12, false);
        PivotSubsystem.getInstance().setPivotSpeed(0);
        System.out.println("Zeroing Pivot!");
    }


    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            PivotSubsystem.getInstance().setPositionHardstop();
        }
        // LEDSubsystem.getInstance().setColor(interrupted ? StatusColors.ERROR : StatusColors.OK);
    }

    @Override
    public boolean isFinished() {
        return PivotSubsystem.getInstance().getRotorVelocity() == 0;

        // TODO check for rotor velocity == 0 after a small period of time so it can fall
    }
}
package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.PhysicalConstants.ElevatorConstants;
import frc.robot.constants.VirtualConstants.ScoringConstants;

/** A command that pushes the elevator down to the hardstop to zero it */
public class ZeroElevatorCommand extends Command {
    double lastRotorVelocity;

    /** Creates a new ZeroElevatorCommand. */
    public ZeroElevatorCommand() {
        setName("ZeroElevatorCommand");

        addRequirements(ElevatorSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        ElevatorSubsystem.getInstance().setVoltage(-ElevatorConstants.zeroElevatorVoltage);
    }


    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        ElevatorSubsystem.getInstance().setVoltage(0);
        if (!interrupted) {
            ElevatorSubsystem.getInstance().setPosition(ScoringConstants.INTAKING_HEIGHT);
        }
        // LEDSubsystem.getInstance().setColor(interrupted ? StatusColors.ERROR : StatusColors.OK);
    }

    @Override
    public boolean isFinished() {
        return ElevatorSubsystem.getInstance().getRotorVelocity() == 0
            && ElevatorSubsystem.getInstance().getStatorCurrent() >= ElevatorConstants.statorCurrentLimit;
    }
}
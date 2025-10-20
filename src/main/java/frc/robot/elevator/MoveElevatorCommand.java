package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VirtualConstants.ElevatorPositions;
import frc.robot.constants.VirtualConstants.PivotPositionNames;
import frc.robot.constants.VirtualConstants.PivotPositions;
import frc.robot.pivot.MovePivotCommand;
import frc.robot.pivot.PivotSubsystem;

import java.util.function.Supplier;

/** A command that moves the elevator to a position. */
public class MoveElevatorCommand extends Command {
    private final double position;
    private final Supplier<Boolean> slowSupplier;
    private final boolean returnToIdle;

    /**
     * Creates a new ElevatorCommand.
     * @param position - The position the elevator will move to. In meters
     * @param slowSupplier - The supplier for running the elevator slower.
     * @param returnToIdle - When the command ends, the elevator will return to the idle position.
     * It will also make the command never end, such that it stays at the position until interrupted.
     */
    public MoveElevatorCommand(double position, Supplier<Boolean> slowSupplier, boolean returnToIdle) {
        setName("MoveElevatorCommand");

        this.position = position;
        this.slowSupplier = slowSupplier;
        this.returnToIdle = returnToIdle;

        addRequirements(ElevatorSubsystem.getInstance());
    }

    /**
     * Creates a new ElevatorCommand.
     * @param position - The position the elevator will move to in meters.
     * If Double.NaN, it will move to the current position.
     * @param slow - Whether to move the elevator slower.
     * @param returnToIdle - When the command ends, the elevator will return to the bottom position.
     * It will also make the command never end, such that it stays at the position until interrupted.
     */
    public MoveElevatorCommand(double position, boolean slow, boolean returnToIdle) {
        this(position, () -> slow, returnToIdle); // Still creates a supplier that supplies just the boolean
    }

    private void pivotSafety() {
        double currentPos = PivotSubsystem.getInstance().getPosition();
        double goalPos = position; // reassign for clarity

        if(
            (currentPos < ElevatorPositions.SAFE_PIVOT_HEIGHT && goalPos > ElevatorPositions.SAFE_PIVOT_HEIGHT) // up
                || (currentPos > ElevatorPositions.SAFE_PIVOT_HEIGHT && goalPos < ElevatorPositions.SAFE_PIVOT_HEIGHT) // down
        ) {
            new MovePivotCommand(PivotPositions.ELEVATING, PivotPositionNames.ELEVATING);
        }
    }

    @Override
    public void initialize() {
        if(!PivotSubsystem.getInstance().isSafeToElevate()) {
            pivotSafety();
        }
    }

    @Override
    public void execute() {
        if(PivotSubsystem.getInstance().isSafeToElevate()){
            ElevatorSubsystem.getInstance().motionMagicPosition(
                Double.isNaN(this.position) ? ElevatorSubsystem.getInstance().getPosition()
                    : this.position, true, this.slowSupplier.get()
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (this.returnToIdle) {
            if(!PivotSubsystem.getInstance().isSafeToElevate()) {
                pivotSafety();
            }

            ElevatorSubsystem.getInstance().motionMagicPosition(
                ElevatorPositions.IDLE_HEIGHT, true, this.slowSupplier.get()
            );
        }
    }

    @Override
    public boolean isFinished() {
        return !this.returnToIdle
            && (
            Double.isNaN(this.position)
                || ElevatorSubsystem.getInstance().withinTolerance(this.position)
        );
    }
}
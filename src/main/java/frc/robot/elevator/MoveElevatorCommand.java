package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.pivot.PivotSubsystem;

import java.util.function.Supplier;

/** A command that moves the elevator to a position. */
public class MoveElevatorCommand extends Command {
    private final double position;
    private final Supplier<Boolean> slowSupplier;
    private boolean safe;

    /**
     * Creates a new ElevatorCommand.
     * @param position - The position the elevator will move to. In meters
     * @param slowSupplier - The supplier for running the elevator slower.
     */
    public MoveElevatorCommand(double position, Supplier<Boolean> slowSupplier) {
        setName("MoveElevatorCommand");

        this.position = position;
        this.slowSupplier = slowSupplier;

        addRequirements(ElevatorSubsystem.getInstance());
    }

    /**
     * Creates a new ElevatorCommand.
     * @param position - The position the elevator will move to in meters.
     * @param slow - Whether to move the elevator slower.
     */
    public MoveElevatorCommand(double position, boolean slow) {
        this(position, () -> slow); // Still creates a supplier that supplies just the boolean
    }

    @Override
    public void initialize() {
        safe = PivotSubsystem.getInstance().isSafeToElevate();
    }

    @Override
    public void execute() {
        if(safe){
            ElevatorSubsystem.getInstance().motionMagicPosition(this.position, true, this.slowSupplier.get());
        } else {
            safe = PivotSubsystem.getInstance().isSafeToElevate();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return ElevatorSubsystem.getInstance().withinTolerance(this.position);
    }
}
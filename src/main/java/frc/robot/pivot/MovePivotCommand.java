// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.PivotConstants;

/** Moves the pivot to a set location */
public class MovePivotCommand extends Command {
    private final double position;

    /**
     * Creates a new PivotCommand.
     * @param position The position to pivot to. Used as a fallback if position cannot be calculated.
     * @apiNote The position is clamped by the soft limits in {@link PivotConstants}.
     */
    public MovePivotCommand(double position) {
        setName("PivotCommand");

        this.position = position;

        addRequirements(PivotSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        PivotSubsystem.getInstance().motionMagicPosition(this.position, false);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        System.out.println(!interrupted ? "Pivot went to " + position : "Pivot move was interrupted");
    }

    @Override
    public boolean isFinished() {
        return PivotSubsystem.getInstance().withinTolerance(this.position);
    }
}
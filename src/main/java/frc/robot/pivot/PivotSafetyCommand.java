// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ElevatorConstants;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.constants.VirtualConstants.PivotPositionNames;
import frc.robot.constants.VirtualConstants.PivotPositions;
import frc.robot.elevator.ElevatorSubsystem;

/** Moves the pivot to a position that's safe to elevate with */
public class PivotSafetyCommand extends Command {
    private double goalPos;
    private double currentPos;
    private boolean end;

    /**
     * Moves the Pivot out of the way of the elevator
     * @param goalPos the position the elevator will go to
     **/
    public PivotSafetyCommand(double goalPos) {
        setName("PivotSafetyCommand");

        this.goalPos = goalPos;

        addRequirements(PivotSubsystem.getInstance());
    }

    /**
     * Moves the Pivot out of the way of the elevator
     **/
    public PivotSafetyCommand() {
        this(-1);
    }

    @Override
    public void initialize() {
        // check if pivot move is necessary
        if (goalPos > 0 // -1 if not specified
            && (
                (currentPos < ElevatorConstants.SAFE_PIVOT_HEIGHT && goalPos < ElevatorConstants.SAFE_PIVOT_HEIGHT) // up
                || (currentPos > ElevatorConstants.SAFE_PIVOT_HEIGHT && goalPos > ElevatorConstants.SAFE_PIVOT_HEIGHT) // down
            )
        ) {
            end = true;
        } else {
            PivotSubsystem.getInstance().motionMagicPosition(PivotPositions.ELEVATING);
            PivotSubsystem.getInstance().setPositionName(PivotPositionNames.ELEVATING);
        }
    }

    @Override
    public void execute() {
        currentPos = ElevatorSubsystem.getInstance().getPosition();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return
            (PivotSubsystem.getInstance().getPosition() >= PivotConstants.MINIMUM_SAFE
            && PivotSubsystem.getInstance().getPosition() <= PivotConstants.MAXIMUM_SAFE)
            || end;
    }
}
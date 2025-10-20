// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.constants.VirtualConstants.PivotPositionNames;
import frc.robot.utilities.Elastic;
import frc.robot.utilities.Elastic.Notification;
import frc.robot.utilities.Elastic.NotificationLevel;

/** Moves the pivot to a set location */
public class MovePivotCommand extends Command {
    private final double position;
    private PivotPositionNames positionName;

    /**
     * Creates a new PivotCommand.
     * @param position The position to pivot to. Used as a fallback if position cannot be calculated.
     * @apiNote The position is clamped by the soft limits in {@link PivotConstants}.
     */
    public MovePivotCommand(double position, PivotPositionNames positionName) {
        setName("MovePivotCommand");

        this.position = position;
        this.positionName = positionName;

        addRequirements(PivotSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        PivotSubsystem.getInstance().setPositionName(this.positionName);
        PivotSubsystem.getInstance().motionMagicPosition(this.position);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "Pivot", "Move Command Interrupted"));
        }
    }

    @Override
    public boolean isFinished() {
        return PivotSubsystem.getInstance().withinTolerance(this.position);
    }
}
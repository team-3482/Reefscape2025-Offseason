package frc.robot.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ManipulatorConstants;
import frc.robot.constants.VirtualConstants.PivotPositionNames;
import frc.robot.constants.VirtualConstants.SubsystemStates;
import frc.robot.pivot.PivotSubsystem;
import frc.robot.utilities.Elastic;
import frc.robot.utilities.Elastic.Notification;
import frc.robot.utilities.Elastic.NotificationLevel;

/** A command that outtakes the algae */
public class OuttakeAlgaeCommand extends Command {
    public OuttakeAlgaeCommand() {
        setName("OuttakeAlgaeCommand");
        addRequirements(ManipulatorSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        if(PivotSubsystem.getInstance().getPositionName().equals(PivotPositionNames.ALGAE)) {
            ManipulatorSubsystem.getInstance().setAlgaeMotor(ManipulatorConstants.ALGAE_OUTTAKE_SPEED);
            ManipulatorSubsystem.getInstance().setState("Algae", SubsystemStates.OUTTAKING);
        } else {
            Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Manipulator", "Algae can't outtake at this position"));
        }
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        ManipulatorSubsystem.getInstance().setAlgaeMotor(0);
        ManipulatorSubsystem.getInstance().setState("Algae", SubsystemStates.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
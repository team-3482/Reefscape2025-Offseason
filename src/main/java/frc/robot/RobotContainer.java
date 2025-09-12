// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.VirtualConstants.ControllerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.vision.LimelightSubsystem;

public class RobotContainer {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class RobotContainerHolder {
        private static final RobotContainer INSTANCE = new RobotContainer();
    }

    public static RobotContainer getInstance() {
        return RobotContainerHolder.INSTANCE;
    }

    private final SendableChooser<Command> autoChooser;
    private Command auton = null;

    // Instance of the controllers used to drive the robot
    private final CommandXboxController driverController;
    // private final CommandXboxController operatorController;

    public RobotContainer() {
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);

        // Configure the trigger bindings
        configureDriverBindings();
        configureOperatorBindings();

        initializeSubsystems();

        this.autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        this.autoChooser.onChange((Command autoCommand) -> this.auton = autoCommand); // Reloads the stored auto

        SmartDashboard.putData("Auto Chooser", this.autoChooser);
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
    }

    /** Creates instances of each subsystem so periodic runs on startup. */
    @SuppressWarnings("ResultOfMethodCallIgnored")
    private void initializeSubsystems() {
        LimelightSubsystem.getInstance();
    }

    /** Configures the button bindings of the driver controller. */
    public void configureDriverBindings() {}

    /** Configures the button bindings of the operator controller. */
    public void configureOperatorBindings() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        if (this.auton == null) {
            this.auton = this.autoChooser.getSelected();
        }
        return this.auton;
    }
}

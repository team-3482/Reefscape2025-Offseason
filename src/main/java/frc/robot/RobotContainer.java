// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VirtualConstants.ControllerConstants;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.SwerveTelemetry;

public class RobotContainer {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class RobotContainerHolder {
        private static final RobotContainer INSTANCE = new RobotContainer();
    }

    public static RobotContainer getInstance() {
        return RobotContainerHolder.INSTANCE;
    }

    // private final SendableChooser<Command> autoChooser; // TODO PathPlanner
    private Command auton = null;

    // Instance of the controllers used to drive the robot
    private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
    // private final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);

    public RobotContainer() {
        configureDrivetrain();
        initializeSubsystems();

        /* TODO PathPlanner
        this.autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        this.autoChooser.onChange((Command autoCommand) -> this.auton = autoCommand); // Reloads the stored auto

        SmartDashboard.putData("Auto Chooser", this.autoChooser); */
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
    }

    /** Creates instances of each subsystem so periodic runs on startup. */
    @SuppressWarnings("ResultOfMethodCallIgnored")
    private void initializeSubsystems() {
        // ExampleSubsystem.getInstance();
    }

    /**
     * This method initializes the swerve subsystem and configures its bindings with the driver controller.
     * This is based on the Phoenix6 Swerve example.
     */
    private void configureDrivetrain(){
        final SwerveSubsystem drivetrain = SwerveSubsystem.getInstance();

        final double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
        final double MaxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
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
            this.auton = Commands.none(); //this.autoChooser.getSelected(); // TODO PathPlanner
        }
        return this.auton;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VirtualConstants.ControllerConstants;
import frc.robot.led.LEDSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.SwerveTelemetry;

import java.util.Map;
import java.util.function.Supplier;

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
    private final CommandXboxController driverController;
    private final XboxController driverController_HID;
    // private final CommandXboxController operatorController;
    // private final XboxController operatorController_HID;

    public RobotContainer() {
        // Initialize controllers
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
        this.driverController_HID = this.driverController.getHID();
        // this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);
        // this.operatorController_HID = this.operatorController.getHID();

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
        LEDSubsystem.getInstance();
    }

    /**
     * This method initializes the swerve subsystem and configures its bindings with the driver controller.
     * This is based on the Phoenix6 Swerve example.
     */
    private void configureDrivetrain(){
        final SwerveSubsystem Drivetrain = SwerveSubsystem.getInstance();

        final double NormalSpeed = SwerveConstants.kSpeedNormal.in(Units.MetersPerSecond);
        final double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
        final double NormalAngularSpeed = SwerveConstants.kAngularSpeedNormal.in(Units.RadiansPerSecond);
        final double FastAngularSpeed = SwerveConstants.kAngularSpeedFast.in(Units.RadiansPerSecond);

        /* Setting up bindings for necessary control of the swerve drive platform */
        final SwerveRequest.FieldCentric fieldCentricDrive_withDeadband = new SwerveRequest
            .FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

        // TODO maybe make these increase/decrease the speed depending on how much the trigger is pressed
        Supplier<Boolean> leftTrigger = () -> this.driverController_HID.getLeftTriggerAxis() >= 0.5;
        Supplier<Boolean> rightTrigger = () -> this.driverController_HID.getRightTriggerAxis() >= 0.5;

        // Drivetrain will execute this command periodically
        Drivetrain.setDefaultCommand(
            Drivetrain.applyRequest(() -> {
                boolean topSpeed = leftTrigger.get();
                boolean fineControl = rightTrigger.get();

                double linearSpeed = topSpeed && !fineControl ? MaxSpeed : NormalSpeed;
                double angularSpeed = topSpeed && !fineControl ? FastAngularSpeed : NormalAngularSpeed;

                double fineControlMult = fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1;

                return fieldCentricDrive_withDeadband
                    // Drive forward with negative Y (forward)
                    .withVelocityX(-driverController.getLeftY() * linearSpeed * fineControlMult)
                    // Drive left with negative X (left)
                    .withVelocityY(-driverController.getLeftX() * linearSpeed * fineControlMult)
                    // Drive counterclockwise with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * angularSpeed * fineControlMult)

                    .withDeadband(ControllerConstants.DEADBAND * linearSpeed)
                    .withRotationalDeadband(ControllerConstants.DEADBAND * angularSpeed);
            }).ignoringDisable(true)
        );

        // Useful for testing
        // final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        // this.driverController.y().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
        //     new Rotation2d(
        //         Math.abs(driverController.getLeftY()) >= 0.25 ? -driverController.getLeftY() : 0,
        //         Math.abs(driverController.getLeftX()) >= 0.25 ? -driverController.getLeftX() : 0
        //     )
        // )));

        // POV / D-PAD
        final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        if (ControllerConstants.DPAD_DRIVE_INPUT) {
            /** POV angle : [X velocity, Y velocity] in m/s */
            final Map<Integer, Integer[]> povSpeeds = Map.ofEntries(
                Map.entry(  0, new Integer[]{ 1,  0}),
                Map.entry( 45, new Integer[]{ 1, -1}),
                Map.entry( 90, new Integer[]{ 0, -1}),
                Map.entry(135, new Integer[]{-1, -1}),
                Map.entry(180, new Integer[]{-1,  0}),
                Map.entry(225, new Integer[]{-1,  1}),
                Map.entry(270, new Integer[]{ 0,  1}),
                Map.entry(315, new Integer[]{ 1,  1})
            );

            povSpeeds.forEach(
                (Integer angle, Integer[] speeds) -> this.driverController.pov(angle).whileTrue(
                    Drivetrain.applyRequest(() -> {
                        boolean faster = leftTrigger.get();
                        boolean robotCentric = rightTrigger.get();

                        return robotCentric
                            ? robotCentricDrive
                            .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                            .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25))
                            : fieldCentricDrive
                            .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                            .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25));
                    })
                )
            );
        }

        Drivetrain.registerTelemetry(logger::telemeterize);
    }

    /** Configures the button bindings of the driver controller. */
    public void configureDriverBindings() {
        // Double Rectangle
        this.driverController.back().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().resetPose(Pose2d.kZero)));
        // Burger
        this.driverController.start().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().seedFieldCentric()));
    }

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
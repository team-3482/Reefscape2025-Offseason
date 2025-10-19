// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.constants.VirtualConstants.ScoringConstants;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.MoveElevatorCommand;
import frc.robot.elevator.ZeroElevatorCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.SwerveTelemetry;
import org.littletonrobotics.junction.Logger;

import java.util.Map;
import java.util.function.Supplier;

import static frc.robot.constants.PhysicalConstants.RobotConstants.CAN_BUS;

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
    private final CommandXboxController operatorController;

    public RobotContainer() {
        // Initialize controllers
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
        this.driverController_HID = this.driverController.getHID();
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);

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
        // TODO get rid of this after tuning elevator
        // ElevatorSubsystem.getInstance();
        TalonFX left = new TalonFX(20, CAN_BUS);
        TalonFX right = new TalonFX(21, CAN_BUS);

        left.setControl(new Follower(20, true));
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
                boolean elevatorTooHigh = ElevatorSubsystem.getInstance().getPosition() > ScoringConstants.SLOW_DRIVE_HEIGHT;
                boolean topSpeed = leftTrigger.get();
                boolean fineControl = rightTrigger.get();

                double linearSpeed = elevatorTooHigh
                    ? (SwerveConstants.kElevatorTooHighSpeed.in(Units.MetersPerSecond))
                    : (topSpeed ? MaxSpeed : NormalSpeed);

                Logger.recordOutput("DriveState/MaxSpeed", linearSpeed);
                SmartDashboard.putNumber("DriveState/MaxSpeed", linearSpeed);

                double angularSpeed = topSpeed ? NormalAngularSpeed : FastAngularSpeed;

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
            /* POV angle : [X velocity, Y velocity] in m/s */
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
    public void configureOperatorBindings() {
        this.operatorController.b().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

        // D-PAD: Left -> L1, Down -> L2, Right -> L3, Up -> L4
        Supplier<Boolean> slowElevatorSupplier = () -> this.operatorController.getHID().getRightTriggerAxis() >= 0.5;

        this.operatorController.povLeft()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L1_CORAL, slowElevatorSupplier, true));
        this.operatorController.povDown()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L2_CORAL, slowElevatorSupplier, true));
        this.operatorController.povRight()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L3_CORAL, slowElevatorSupplier, true));
        this.operatorController.povUp()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L4_CORAL, slowElevatorSupplier, true));

        // A -> L2 Algae, Y -> L3 Algae
        this.operatorController.y()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L2_ALGAE, slowElevatorSupplier, true));
        this.operatorController.a()
            .toggleOnTrue(new MoveElevatorCommand(ScoringConstants.L3_ALGAE, slowElevatorSupplier, true));

        // Double Rectangle -> Zero Elevator
        this.operatorController.back().onTrue(new ZeroElevatorCommand());
    }

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
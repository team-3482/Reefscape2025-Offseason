package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.util.Units;

/** Contains physical constants that are related to real-life things.
 * For example, device IDs, speeds, ratios/multipliers, etc.
 * */
public final class PhysicalConstants {
    public static final class RobotConstants {
        /** CAN Bus for all CTRE devices */
        public static final CANBus CAN_BUS = new CANBus("ctre");
    }

    public static final class ElevatorConstants {
        /** The CAN ID for the Left Elevator TalonFX */
        public static final int LEFT_MOTOR_ID = 20;
        /** The CAN ID for the Right Elevator TalonFX */
        public static final int RIGHT_MOTOR_ID = 21;

        /**
         * The diameter of the bar that the string wraps around on the elevator
         * @implSpec This is in meters and is very important to calculate the position of the elevator
         */
        public static final double ROLLER_DIAMETER = Units.inchesToMeters(1.125);

        /** Heuristic offset multiplier if math is wrong */
        public static final double LINEAR_CONSTANT_MULT = 0.75;

        /** Tolerance used for elevator command in meters. */
        public static final double HEIGHT_TOLERANCE = 0.05;

        /** The voltage to run the motors at to zero the elevator */
        public static final double ZERO_ELEVATOR_VOLTAGE = 2;

        /** Elevator Motor Stator Current Limit */
        public static final double STATOR_CURRENT_LIMIT = 80;

        /* Motion Magic Config */
        public static final double ROTOR_TO_MECHANISM_RATIO = 3; // 36:12, 3:1
        public static final double CRUISE_SPEED = 30;
        public static final double SLOW_CRUISE_SPEED = 10;
        public static final double ACCELERATION = 60;
        public static final double SLOW_ACCELERATION = 30;

        /** Gains used for Motion Magic slot 0. */
        public static final class Slot0Gains {
            public static final double kG = 0.37;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 2;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class PivotConstants {
        /** The CAN ID for the Pivot TalonFX */
        public static final int MOTOR_ID = 22;

        /** Lower angle limit */
        public static final double LOWER_ANGLE_LIMIT = -10;
        /** Upper angle limit */
        public static final double UPPER_ANGLE_LIMIT = 270;

        /** Tolerance for Commands using MotionMagic in degrees. */
        public static final double POSITION_TOLERANCE = 0.1;

        /* Motion Magic Config */
        public static final double ROTOR_TO_MECHANISM_RATIO = 61; // 61:1
        public static final double CRUISE_SPEED = 2;
        public static final double ACCELERATION = 4;

        /** Gains used for Motion Magic slot 0. */
        public static final class Slot0Gains {
            public static final double kG = 0;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;
        }

    }
}
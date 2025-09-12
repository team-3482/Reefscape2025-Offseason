// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.Set;

/** Contains virtual constants that do not correlate to anything physical in real life. */
public final class VirtualConstants {
    /** Constants for the controller and any controller-related assignments. */
    public static final class ControllerConstants {
        /** DriverStation ID of the driver controller. */
        public static final int DRIVER_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller. */
        public static final int OPERATOR_CONTROLLER_ID = 1;
        /** Removes input around the joystick's center (eliminates stick drift). */
        public static final double DEADBAND = 0.075;
        /** Whether to accept directional pad input for movement. */
        public static final boolean DPAD_DRIVE_INPUT = true;
        /** Speed multiplier when using fine-control movement. */
        public static final double FINE_CONTROL_MULT = 0.15;
    }

    /** Tab names in Elastic. */
    public static final class DashboardTabNames {
        public static final String TELEOP = "Teleoperated";
        public static final String AUTON = "Autonomous";
        public static final String DEV = "Dev";
    }

    /** Constants for Limelight-related code. */
    public static final class LimelightConstants {
        /** Enables publishing of the CameraFeeds to dashboard on startup. */
        public static final boolean PUBLISH_CAMERA_FEEDS = true;

        /** Bottom Limelight (Reef). */
        public static final String BOTTOM_RIGHT_LL = "limelight-stheno";
        /** Top Limelight (Processor/Barge). */
        public static final String BOTTOM_LEFT_LL = "limelight-euryale";

        /**
         * The distance within which to use Limelight data in meters. This is measured from tag to camera.
         * When entering this range, the LimelightSubsystem will reset the odometry to the current LL pose.
         */
        public static final double REEF_TRUST_RANGE = 1.5;
        public static final double TRUST_RANGE = 4;
        public static final double REEF_ALIGN_RANGE = 2.0;

        /** All valid tag IDs (used for tag filtering) */
        public static final int[] ALL_TAG_IDS = new int[]{
                1, 2, 3, 4, 5,
                6, 7, 8, 9, 10,
                11, 12, 13, 14,
                15, 16, 17, 18,
                19, 20, 21, 22
        };

        /**
         * Crop window size when no tags are in view (used for smart cropping).
         * {@code xMin, xMax, yMin, yMax}.
         */
        public static final double[] DEFAULT_BOTTTOM_CROP = new double[]{-1, 1, -1, 1};

        /** Horizontal resolution of limelight in pixels. */
        public static final double RES_X = 1280;
        /** Vertical resolution of limelight in pixels. */
        public static final double RES_Y = 800;

        /** Increase any crop by this value around a pixel extremity. */
        public static final double BOUNDING_BOX = 0.3;
    }

    /** Constants used for auto-aligning the robot when scoring. */
    public static final class AligningConstants {
        public static final class Reef {
            /** How far (in meters) the robot should be from the tag perpendicularly to be flush with the reef. */
            public static final double PERPENDICULAR_DIST_TO_TAG = 0.44;
            /** Coral distance added before the real lign up in meters. */
            public static final double CORAL_DISTANCE = 0.11 /* Coral diameter */;
            /** How far (in meters) the robot should be parallel to the tag to score. */
            public static final double PARALLEL_DIST_TO_TAG = 0.18;
        }

        @Deprecated
        public static final class Processor {
            /** How far (in meters) the robot should be from the tag perpendicularly to score. */
            public static final double PERPENDICULAR_DIST_TO_TAG = 0.7;
            /** How far (in meters) the robot should be parallel to the tag to score. */
            public static final double PARALLEL_DIST_TO_TAG = 0.0;
        }
    }

    /** Sets of tags for O(1) lookup. */
    public static final class TagSets {
        public static final Set<Integer> REEF_TAGS_NORMAL = Set.of(6, 7, 8, 17, 18, 19);
        public static final Set<Integer> REEF_TAGS_FLIPPED = Set.of(9, 10, 11, 20, 21, 22);
        public static final Set<Integer> REEF_TAGS = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
        public static final Set<Integer> EMPTY_SET = Set.of();

        @Deprecated
        public static final Set<Integer> PROCESSOR_TAGS = Set.of(3, 16);
        @Deprecated
        public static final Set<Integer> BARGE_TAGS = Set.of(4, 5, 14, 15);
    }
}
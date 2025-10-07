package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import java.util.Set;

import static edu.wpi.first.units.Units.Inches;

/** Constants for vision subsystems */
public final class VisionConstants {
    /** Constants for Limelight-related code. */
    public static final class LimelightConstants {
        /** Enables publishing of the CameraFeeds to dashboard on startup. */
        public static final boolean PUBLISH_CAMERA_FEEDS = true;

        /** Bottom Limelight (Reef) */
        public static final String BOTTOM_LL = "limelight-stheno";

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

    /** Constants used by QuestNav */
    public static final class QuestNavConstants {
        /**
         * Quest mounting offsets
         * X (positive) -> forward from robot center
         * Y (positive) -> left from robot center
         * Rotation (positive) -> counter-clockwise
         */
        public static final Transform2d ROBOT_TO_QUEST = new Transform2d(
            new Translation2d(
                Inches.of(13), // TODO measure this again from the center of the bot to the lenses, not the cameras
                Inches.of(0) // Centered
            ),
            new Rotation2d() // Facing forward
        );

        /** Standard Deviations for Trust */
        public static final Matrix<N3, N1> TRUST_STD_DEVS = VecBuilder.fill(
            0.02, // Trust down to 2cm in X direction
            0.02, // Trust down to 2cm in Y direction
            Units.degreesToRadians(2) // Trust down to 2 degrees rotational
        );
    }
}
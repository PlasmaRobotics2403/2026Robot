package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static String camera0Name = "camera_0";
    public static String camera1Name = "camera_1";

    public static Transform3d robotToCamera0 = new Transform3d(
            -0.2953512, -0.21550884, 0.49809146, new Rotation3d(0.0, Math.toRadians(70), Math.toRadians(180)));
    public static Transform3d robotToCamera1 = new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    public static double[] cameraStdDevFactors = new double[] {1.0, 1.0};

    public static double linearStdDevMegatag2Factor = 0.5;
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}

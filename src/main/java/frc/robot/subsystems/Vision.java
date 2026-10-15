package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Field;

/**
 * PhotonVision AprilTag localization. Feeds every good measurement into the pose estimator.
 *
 * <p>Each camera publishes a plain-English status to SmartDashboard ("Vision/Front Status",
 * "Vision/Side Status") saying why it is or isn't updating the pose, plus its raw estimate before
 * filtering ("Vision/Side/RawPose") so a bad robot-to-camera transform is easy to spot.
 *
 * <p>For the CarboVis match view, every loop each camera also logs its mount (RobotToCamera), health
 * (Connected, TagCount, LatencyMs, FPS) and the tags in its latest result (TagIds, TagYawDeg,
 * TagDistanceM: same length and order, empty when none, yaw CCW+ so a tag on the LEFT is positive).
 */
public final class Vision {
    /** Where accepted measurements go (the drivetrain's pose estimator). */
    @FunctionalInterface
    public interface Consumer {
        void accept(Pose2d pose, double timestampS, Matrix<N3, N1> stdDevs);
    }

    private static final class Camera {
        final String label;
        final PhotonCamera camera;
        final PhotonPoseEstimator estimator;
        final double[] robotToCamera;
        int accepted = 0;
        String status = "starting";
        // Tags in the latest result, re-logged every loop until a new result arrives.
        long[] tagIds = new long[0];
        double[] tagYawDeg = new double[0];
        double[] tagDistanceM = new double[0];
        double latencyMs = 0;
        double fps = 0;
        long lastCaptureMicros = 0;

        Camera(String label, String photonName, Transform3d robotToCamera) {
            this.label = label;
            camera = new PhotonCamera(photonName);
            estimator = new PhotonPoseEstimator(Field.TAGS, robotToCamera);
            this.robotToCamera = mountArray(robotToCamera);
        }

        void recordTags(PhotonPipelineResult result) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            int n = targets.size();
            tagIds = new long[n];
            tagYawDeg = new double[n];
            tagDistanceM = new double[n];
            for (int i = 0; i < n; i++) {
                Transform3d t = targets.get(i).getBestCameraToTarget();
                tagIds[i] = targets.get(i).getFiducialId();
                tagYawDeg[i] = tagYawDeg(t);
                tagDistanceM[i] = tagFloorDistanceM(t);
            }
            latencyMs = result.metadata.getLatencyMillis();
            long capture = result.metadata.getCaptureTimestampMicros();
            if (lastCaptureMicros > 0 && capture > lastCaptureMicros) fps = 1e6 / (capture - lastCaptureMicros);
            lastCaptureMicros = capture;
        }

        void clearTags() {
            tagIds = new long[0];
            tagYawDeg = new double[0];
            tagDistanceM = new double[0];
        }
    }

    /** Horizontal angle from the camera's center line to the tag, degrees, CCW+ (tag on the LEFT is positive). */
    static double tagYawDeg(Transform3d cameraToTarget) {
        return Math.toDegrees(Math.atan2(cameraToTarget.getY(), cameraToTarget.getX()));
    }

    /** Floor-plane distance from the camera to the tag, meters. */
    static double tagFloorDistanceM(Transform3d cameraToTarget) {
        return Math.hypot(cameraToTarget.getX(), cameraToTarget.getY());
    }

    /** {x m, y m, z m, roll deg, pitch deg, yaw deg}, robot center to camera. */
    static double[] mountArray(Transform3d m) {
        return new double[] {m.getX(), m.getY(), m.getZ(), Math.toDegrees(m.getRotation().getX()),
                Math.toDegrees(m.getRotation().getY()), Math.toDegrees(m.getRotation().getZ())};
    }

    private final List<Camera> cameras = List.of(
            new Camera("Front", Constants.Vision.CAM_FRONT, Constants.Vision.ROBOT_TO_CAM_FRONT),
            new Camera("Side", Constants.Vision.CAM_SIDE, Constants.Vision.ROBOT_TO_CAM_SIDE));

    public Vision() {
        for (Camera cam : cameras) {
            Health.addCheck(() -> cam.camera.isConnected() ? null : cam.label + " camera NOT CONNECTED");
        }
    }

    public void addMeasurements(Consumer poseEstimator) {
        for (Camera cam : cameras) {
            boolean connected = cam.camera.isConnected();
            if (!connected) {
                cam.status = "NOT CONNECTED (camera name in PhotonVision must match Constants.Vision)";
                cam.clearTags(); // don't keep drawing tags from a camera that's gone
            }
            for (PhotonPipelineResult result : cam.camera.getAllUnreadResults()) {
                cam.recordTags(result);
                cam.status = process(cam, result, poseEstimator);
            }
            String key = "Vision/" + cam.label + "/";
            SmartDashboard.putString("Vision/" + cam.label + " Status", cam.status);
            DogLog.log(key + "Status", cam.status);
            DogLog.log(key + "Accepted", cam.accepted);
            DogLog.log(key + "RobotToCamera", cam.robotToCamera);
            DogLog.log(key + "Connected", connected);
            DogLog.log(key + "TagCount", (double) cam.tagIds.length);
            DogLog.log(key + "LatencyMs", cam.latencyMs, "ms");
            DogLog.log(key + "FPS", cam.fps);
            DogLog.log(key + "TagIds", cam.tagIds);
            DogLog.log(key + "TagYawDeg", cam.tagYawDeg, "deg");
            DogLog.log(key + "TagDistanceM", cam.tagDistanceM, "m");
        }
    }

    /** @return status text for this result */
    private static String process(Camera cam, PhotonPipelineResult result, Consumer poseEstimator) {
        if (!result.hasTargets()) return "connected, no tags in view";

        Optional<EstimatedRobotPose> maybe = cam.estimator.estimateCoprocMultiTagPose(result);
        if (maybe.isEmpty()) maybe = cam.estimator.estimateLowestAmbiguityPose(result);
        if (maybe.isEmpty()) return "sees tags but no 3D pose (is the camera calibrated at this resolution?)";

        EstimatedRobotPose est = maybe.get();
        Pose3d pose = est.estimatedPose;
        DogLog.log("Vision/" + cam.label + "/RawPose", pose);

        List<PhotonTrackedTarget> tags = est.targetsUsed;
        int count = tags.size();
        double avgDistance = 0;
        for (PhotonTrackedTarget tag : tags) {
            avgDistance += tag.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDistance /= Math.max(1, count);

        if (Math.abs(pose.getZ()) > Constants.Vision.MAX_Z_ERROR_M) {
            return String.format("REJECTED: pose %.2f m off the floor (check robot-to-camera transform)", pose.getZ());
        }
        if (pose.getX() < 0 || pose.getX() > Field.LENGTH_M || pose.getY() < 0 || pose.getY() > Field.WIDTH_M) {
            return "REJECTED: pose outside the field (check robot-to-camera transform)";
        }
        if (count == 1 && tags.get(0).getPoseAmbiguity() > Constants.Vision.MAX_AMBIGUITY) {
            return String.format("REJECTED: single tag too ambiguous (%.2f)", tags.get(0).getPoseAmbiguity());
        }
        if (count == 1 && avgDistance > Constants.Vision.MAX_SINGLE_TAG_DIST_M) {
            return String.format("REJECTED: single tag too far (%.1f m)", avgDistance);
        }

        double[] base = count > 1 ? Constants.Vision.MULTI_TAG_STD : Constants.Vision.SINGLE_TAG_STD;
        double scale = 1 + avgDistance * avgDistance / 30;
        poseEstimator.accept(pose.toPose2d(), est.timestampSeconds,
                VecBuilder.fill(base[0] * scale, base[1] * scale, base[2] * scale));
        cam.accepted++;
        DogLog.log("Vision/" + cam.label + "/Pose", pose);
        return "OK: " + count + (count == 1 ? " tag" : " tags");
    }
}

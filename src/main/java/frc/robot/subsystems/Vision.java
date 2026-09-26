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
        int accepted = 0;
        String status = "starting";

        Camera(String label, String photonName, Transform3d robotToCamera) {
            this.label = label;
            camera = new PhotonCamera(photonName);
            estimator = new PhotonPoseEstimator(Field.TAGS, robotToCamera);
        }
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
            if (!cam.camera.isConnected()) {
                cam.status = "NOT CONNECTED (camera name in PhotonVision must match Constants.Vision)";
            }
            for (PhotonPipelineResult result : cam.camera.getAllUnreadResults()) {
                cam.status = process(cam, result, poseEstimator);
            }
            SmartDashboard.putString("Vision/" + cam.label + " Status", cam.status);
            DogLog.log("Vision/" + cam.label + "/Status", cam.status);
            DogLog.log("Vision/" + cam.label + "/Accepted", cam.accepted);
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

package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionSubsystem {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout; 

    public VisionSubsystem() {
        camera = new PhotonCamera("Camera_Module_v3");

        // O yılın resmi saha dizilimini yükle
        try {
            fieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
        } catch (Exception e) {
            throw new RuntimeException("AprilTag haritası yüklenemedi!", e);
        }

        Transform3d robotToCam = new Transform3d(
                new Translation3d(-0.2, 0.0, 0.1), // Kameranın merkezden konumu (X, Y, Z)
                new Rotation3d(
                        0,                       // Roll (X Ekseni): Kamera dik duruyor
                        Math.toRadians(-15),     // Pitch (Y Ekseni): 15 derece yukarı bakıyor
                        Math.toRadians(180)      // Yaw (Z Ekseni): 180 derece arkaya bakıyor
                )
        );

        poseEstimator = new PhotonPoseEstimator(
                fieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                robotToCam
        );
    }

    // İstenilen Tag ID'sini kamerada arar ve merkezden ne kadar sapmış (Yaw) onu verir.
    public Optional<Double> getTargetYaw(int targetId) {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == targetId) {
                    return Optional.of(target.getYaw());
                }
            }
        }
        return Optional.empty(); 
    }

    // YENİ: İstenilen Tag ID'sinin kamerada kapladığı alanı (% olarak) verir. Uzaklık için!
    public Optional<Double> getTargetArea(int targetId) {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (result.hasTargets()) {
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == targetId) {
                    return Optional.of(target.getArea()); 
                }
            }
        }
        return Optional.empty(); 
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        PhotonPipelineResult result = camera.getLatestResult();
        return poseEstimator.update(result);
    }

    public Optional<Pose3d> getTagPose(int tagID) {
        if (fieldLayout == null) return Optional.empty();
        return fieldLayout.getTagPose(tagID);
    }
}
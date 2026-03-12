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
import edu.wpi.first.math.geometry.Pose3d; // BUNU EKLE

public class VisionSubsystem {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout; // SINIF SEVİYESİNE TAŞINDI

    public VisionSubsystem() {
        camera = new PhotonCamera("MainCam");

        // O yılın resmi saha dizilimini yükle
        try {
            fieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
        } catch (Exception e) {
            throw new RuntimeException("AprilTag haritası yüklenemedi!", e);
        }

        Transform3d robotToCam = new Transform3d(
                new Translation3d(0.2, 0.0, 0.15), 
                new Rotation3d(0, 0, 0)
        );

        poseEstimator = new PhotonPoseEstimator(
                fieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                robotToCam
        );
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        PhotonPipelineResult result = camera.getLatestResult();
        return poseEstimator.update(result);
    }

    // YENİ METOD: İstediğimiz ID'ye sahip AprilTag'in sahadaki mutlak konumunu verir
    public Optional<Pose3d> getTagPose(int tagID) {
        if (fieldLayout == null) return Optional.empty();
        return fieldLayout.getTagPose(tagID);
    }
}
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
// YENİ EKLENEN: İstenilen Tag ID'sini kamerada arar ve merkezden ne kadar sapmış (Yaw) onu verir.
    public Optional<Double> getTargetYaw(int targetId) {
        PhotonPipelineResult result = camera.getLatestResult();
        
        // Eğer kamera bir şeyler görüyorsa
        if (result.hasTargets()) {
            // Gördüğü tüm hedefleri tara
            for (var target : result.getTargets()) {
                // Eğer gördüğü hedef aradığımız ID ise (Örn: 27)
                if (target.getFiducialId() == targetId) {
                    return Optional.of(target.getYaw()); // Sapma açısını derece cinsinden döndür
                }
            }
        }
        return Optional.empty(); // Göremiyorsa boş döndür
    }
    public VisionSubsystem() {
        camera = new PhotonCamera("ashina_Port_1182_Output_MJPEG_Server");

        // O yılın resmi saha dizilimini yükle
        try {
            fieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
        } catch (Exception e) {
            throw new RuntimeException("AprilTag haritası yüklenemedi!", e);
        }

// DİKKAT: Eğer kamera robotun merkezine göre arka tarafa monte edildiyse, 
        // Translation3d içindeki X değerini de eksi yapmalısın (Örn: -0.2).
        // Eğer fiziksel olarak önde ama arkaya bakıyorsa 0.2 olarak kalabilir.
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
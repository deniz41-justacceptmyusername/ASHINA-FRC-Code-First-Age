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

public class VisionSubsystem {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public VisionSubsystem() {
         // PhotonVision arayüzünde verdiğin kamera adını gir
        camera = new PhotonCamera("Camera_Module_v3");

        // O yılın resmi saha dizilimini yükle
        AprilTagFieldLayout fieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();

        // Kameranın robot merkezine göre konumunu tanımla (Örnek: 20cm önde, 15cm yukarıda, açısız)
        Transform3d robotToCam = new Transform3d(
                new Translation3d(0.2, 0.0, 0.07), 
                new Rotation3d(0, 0, 0)
        );

        // Tahmin ediciyi oluştur. MULTI_TAG_PNP stratejisi birden fazla tag gördüğünde çok daha isabetlidir.
        poseEstimator = new PhotonPoseEstimator(
                fieldLayout, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                robotToCam
        );
    }

    // Bu metodu robotun periyodik döngüsünde çağırarak anlık konumu alabilirsin
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        PhotonPipelineResult result = camera.getLatestResult();
        return poseEstimator.update(result);
    }
}

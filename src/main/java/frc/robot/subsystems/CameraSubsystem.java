package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase{
    PhotonCamera camera = new PhotonCamera("My Camera");
    public CameraSubsystem() {
    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets()){ // ortamda hedef varsa
     PhotonTrackedTarget target = result.getBestTarget(); // en büyük ve merkezteki hedefi alır

    int targetId = target.getFiducialId(); // april tag ID alır

    Transform3d pose = target.getBestCameraToTarget(); // kameranın 3 boyutlu konumu verir

        double x = pose.getX(); // İleri-geri mesafe (metre)
        double y = pose.getY(); // Sağ-sol mesafe (metre)
        double yaw = target.getYaw(); // Sapma açısı (derece)

    }
    List<PhotonTrackedTarget> targets = result.getTargets();
for (PhotonTrackedTarget t : targets) {
    System.out.println("Tag ID" + t.getFiducialId()); // ortamda birden çok hedef varsa
}
    }
}

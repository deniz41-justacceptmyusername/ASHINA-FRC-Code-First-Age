package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation; // İttifak rengi için eklendi
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.EstimatedRobotPose; // Vizyon için eklendi

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// --- GÜNCEL PATHPLANNER KÜTÜPHANELERİ ---
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SubsystemBase {
  private final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  private final SwerveDrive swerveDrive;
  private final Field2d m_field = new Field2d();
  
  // Vizyon sistemini tutacağımız değişken
  private final VisionSubsystem vision; 

  // CONSTRUCTOR GÜNCELLENDİ: Artık VisionSubsystem istiyor
  public SwerveSubsystem(VisionSubsystem vision) {
    this.vision = vision; // RobotContainer'dan gelen vizyonu kaydet
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false); 
    swerveDrive.setCosineCompensator(false);

    // PathPlanner konfigürasyonunu çağırıyoruz
    setupPathPlanner();
  }

  // YENİ METOT: Güncel PathPlanner Ayarları
  public void setupPathPlanner() {
    // 1. Arayüzden (GUI) robot ayarlarını çek
    RobotConfig config;
    try {
        config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        e.printStackTrace();
        return; // Ayarlar okunamadıysa çökmesini engelle
    }

    // 2. Yeni sisteme göre AutoBuilder'ı yapılandır
    AutoBuilder.configure(
        swerveDrive::getPose, // Robotun anlık konumu
        swerveDrive::resetOdometry, // Konumu sıfırlama
        swerveDrive::getRobotVelocity, // Robotun şasi hızını okuma
        (speeds, feedforwards) -> swerveDrive.setChassisSpeeds(speeds), // YAGSL sürüş metodu
        new PPHolonomicDriveController( // Yeni sürüş kontrolcüsü
            new PIDConstants(5.0, 0.0, 0.0), // X ve Y ekseni PID ayarları
            new PIDConstants(5.0, 0.0, 0.0)  // Dönüş (Rotation) PID ayarları
        ),
        config, // Yukarıda okuduğumuz RobotConfig ayarları
        () -> {
            // Kırmızı ittifaktayken rotayı otomatik aynalar
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this // Alt sistemi zorunlu kılar
    );
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }
  
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }
  
  public void flipGyro180() {
    Pose2d currentPose = swerveDrive.getPose(); 
    Rotation2d invertedRotation = currentPose.getRotation().plus(Rotation2d.fromDegrees(180));
    swerveDrive.resetOdometry(new Pose2d(currentPose.getTranslation(), invertedRotation));
  }
  
  public void robotPeriodic() {
    SwerveDriveTelemetry.updateData();
  }

  @Override
  public void periodic() {
    // 1. YAGSL'ın kendi tekerlek/gyro odometrisini güncelle
    swerveDrive.updateOdometry();

    // 2. VİZYON GÜNCELLEMESİ (YENİ EKLENEN KISIM)
    if (vision != null) {
        Optional<EstimatedRobotPose> visionPose = vision.getEstimatedGlobalPose();
        
        // Sadece kamera bir etiket görüyorsa güncelle!
        if (visionPose.isPresent()) {
            Pose2d camPose2d = visionPose.get().estimatedPose.toPose2d();
            double timestamp = visionPose.get().timestampSeconds;
            
            // YAGSL'ın GİZLİ SİLAHI: Kendi pose estimator'ına vizyonu doğrudan besliyoruz
            swerveDrive.addVisionMeasurement(camPose2d, timestamp);
        }
    }
    
    // 3. Haritayı güncellenmiş (tekerlek + kamera) kusursuz konumla besle
    m_field.setRobotPose(swerveDrive.getPose());
    
    // Telemetry ve Dashboard güncellemeleri
    SwerveDriveTelemetry.updateData();
    SmartDashboard.putData("Field", m_field);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose) {

@Override
  public void periodic() {
    // 1. Simülasyonun arka planda çalışması ve fizik hesaplaması için bu ŞART:
    swerveDrive.updateOdometry(); 
    
    // 2. Bizi çökerten YAGSL telemetrisi KAPALI KALMAYA DEVAM EDİYOR:
    // SwerveDriveTelemetry.updateData(); 
    
    // 3. YENİ KISIM: Robotun koordinatlarını al ve güvenli haritaya (Field2d) yansıt!
    m_field.setRobotPose(swerveDrive.getPose());
    SmartDashboard.putData("Field", m_field);
  }

  public void resetOdometry(edu.wpi.first.math.geometry.Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }
}
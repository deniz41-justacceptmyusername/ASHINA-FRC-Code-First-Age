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
import edu.wpi.first.wpilibj.DriverStation; 
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.EstimatedRobotPose; 

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

  public void setupPathPlanner() {
    RobotConfig config;
    try {
        config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        e.printStackTrace();
        return; 
    }

    AutoBuilder.configure(
        swerveDrive::getPose, 
        swerveDrive::resetOdometry, 
        swerveDrive::getRobotVelocity, 
        (speeds, feedforwards) -> swerveDrive.setChassisSpeeds(speeds), 
        new PPHolonomicDriveController( 
            new PIDConstants(5.0, 0.0, 0.0), 
            new PIDConstants(5.0, 0.0, 0.0)  
        ),
        config, 
        () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this 
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
    // ÇÖKMEYİ ENGELLEMEK İÇİN YORUM SATIRINDA KALIYOR
    // SwerveDriveTelemetry.updateData();
  }

  @Override
  public void periodic() {
    // 1. YAGSL'ın kendi tekerlek/gyro odometrisini güncelle
    swerveDrive.updateOdometry();

    // 2. VİZYON GÜNCELLEMESİ (Eğer vizyon varsa)
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
    SmartDashboard.putData("Field", m_field);
    
    // BİZİ ÇÖKERTEN YAGSL TELEMETRİSİ KAPALI KALMAYA DEVAM EDİYOR
    // SwerveDriveTelemetry.updateData();
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }
}
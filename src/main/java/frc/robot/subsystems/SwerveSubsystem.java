package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation; // İttifak rengi için eklendi
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

// --- GÜNCEL PATHPLANNER KÜTÜPHANELERİ ---
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SubsystemBase {
  private final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  private final SwerveDrive swerveDrive;

  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;

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
  
  public void robotPeriodic() {
    SwerveDriveTelemetry.updateData();
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    SwerveDriveTelemetry.updateData();
  }

  public void resetOdometry(edu.wpi.first.math.geometry.Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }
}
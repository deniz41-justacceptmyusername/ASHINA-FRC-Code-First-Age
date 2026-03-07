package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveSubsystem extends SubsystemBase {
  private final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  private final SwerveDrive swerveDrive;
  private final Field2d m_field = new Field2d();
  

  public SwerveSubsystem() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      // Çevrim katsayılarını sildik, artık her şeyi physicalproperties.json'dan 6.75 ve 26 olarak okuyacak!
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Absolute encoder olmadığı için kritik ayarlar
    swerveDrive.setHeadingCorrection(false); 
    swerveDrive.setCosineCompensator(false);
  }

  // RobotContainer'ın kullanacağı ana sürüş metodu
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
    // Robotun anlık pozisyonunu ve açısını alıyoruz
    Pose2d currentPose = swerveDrive.getPose(); 
    
    // Mevcut açıya 180 derece (Math.PI radyan) ekliyoruz
    Rotation2d invertedRotation = currentPose.getRotation().plus(Rotation2d.fromDegrees(180));
    
    // Konumu aynı bırakıp, açıyı tersine çevrilmiş haliyle güncelliyoruz
    swerveDrive.resetOdometry(new Pose2d(currentPose.getTranslation(), invertedRotation));
}
  
  public void robotPeriodic() {
    SwerveDriveTelemetry.updateData();
  }
@Override
  public void periodic() {
    // Odometry güncellemesi
    swerveDrive.updateOdometry();
    m_field.setRobotPose(swerveDrive.getPose());
    
    // Telemetry ve Dashboard güncellemeleri
    SwerveDriveTelemetry.updateData();
    SmartDashboard.putData("Field", m_field);
  }
public Pose2d getPose() {
    return swerveDrive.getPose();
}
 //Robotun konumunu manuel ayarlamak için
  public void resetOdometry(edu.wpi.first.math.geometry.Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }
   //robotPeriodic() ve subsystemPeriodic() metodlarını silebilirsin.
  
}
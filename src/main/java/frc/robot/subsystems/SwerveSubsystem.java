package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.util.function.Supplier;


import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveSubsystem extends SubsystemBase {

  private final File directory =
      new File(Filesystem.getDeployDirectory(), "swerve");
  private final SwerveDrive swerveDrive;
  
public SwerveSubsystem() {
  this(new File(Filesystem.getDeployDirectory(), "swerve"));
}
  public SwerveSubsystem(File directory) {

    double angleConversationFactor = SwerveMath.calculateDegreesPerSteeringRotation(26, 1);
    double driveConversationFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(2), 7.03, 1);
    
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive =
          new SwerveParser(directory)
              .createSwerveDrive(Constants.maxSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); 

    for(swervelib.SwerveModule m : swerveDrive.getModules()) {
      System.out.println("Module Name: "+m.configuration.name);
      CANcoder absoluteEncoder = (CANcoder)m.configuration.absoluteEncoder.getAbsoluteEncoder();
    }
    // 🔥 NavX kalibrasyon bekleme (çok önemli)
    Timer.delay(4.0);

    // 🔥 Robotun başlangıç yönünü ileri kabul et
    swerveDrive.zeroGyro();
  }

  // Field oriented sürüş (gyro configten geliyor)
  public Command driveFieldOriented(
      Supplier<ChassisSpeeds> speeds) {

    return run(() -> {
      swerveDrive.driveFieldOriented(speeds.get());
    });
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }
}
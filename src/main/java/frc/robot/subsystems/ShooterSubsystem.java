package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

private final TalonFX m_shooterRight = new TalonFX(71, ShooterConstants.kCANBus);
private final TalonFX m_shooterLeft = new TalonFX(72, ShooterConstants.kCANBus);

 private Timer shotTimer = new Timer();
    private boolean isBallInAir = false;

    private double startX = 0.0;
    private double startY = 0.0;
    private double startZ = 0.5;

    private double velocity = 10.0;
    private double pitchAngle = Math.toRadians(45);
    private double yawAngle = Math.toRadians(0);

    private final double GRAVITY = 9.81;

    public void shootBall(double currentRobotX, double currentRobotY, double currentRobotYaw) {
      this.startX = currentRobotX;
      this.startY = currentRobotY;
      this.yawAngle = currentRobotYaw;
        
        isBallInAir = true;
        shotTimer.restart();

    }

private final DutyCycleOut m_request = new DutyCycleOut(0.0);

public ShooterSubsystem() {
    var currentConfigs = new MotorOutputConfigs();
 currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_shooterLeft.getConfigurator().apply(currentConfigs);
        m_shooterRight.getConfigurator().apply(currentConfigs);
}

public void setShooterSpeed(double speed) {
        m_shooterRight.setControl(m_request.withOutput(speed));
        m_shooterLeft.setControl(m_request.withOutput(speed));
}
public void stop() {
        m_shooterRight.stopMotor();
        m_shooterLeft.stopMotor();
  }
  public void simulationPeriodic() {
    if (isBallInAir) {
            double t = shotTimer.get(); // Havada geçen süre (saniye)

            // 1. Z EKSENİ (Yerden Yükseklik)
            // Z = Z_ilk + (V * sin(pitch) * t) - (0.5 * g * t^2)
            double z = startZ + (velocity * Math.sin(pitchAngle) * t) - (0.5 * GRAVITY * Math.pow(t, 2));
            double horizontalDistance = velocity * Math.cos(pitchAngle) * t;

            double x = startX + (horizontalDistance * Math.cos(yawAngle));

            double y = startY + (horizontalDistance * Math.sin(yawAngle));

            if (z < 0.1) { // Top yere değdiyse (yarıçapı kadar mesafe kalmışsa)
                isBallInAir = false;
                shotTimer.stop();
                z = 0.1; // Topun yerin dibine girmesini engelle
            }
            Pose3d simulatedBallPose = new Pose3d(x, y, z, new Rotation3d());
            
            // Loglamak (SmartDashboard veya AdvantageKit Logger ile)
          SmartDashboard.putNumberArray("SimulatedBall", new double[] {
              simulatedBallPose.getX(),
              simulatedBallPose.getY(),
              simulatedBallPose.getZ(),
              simulatedBallPose.getRotation().getQuaternion().getW(),
              simulatedBallPose.getRotation().getQuaternion().getX(),
              simulatedBallPose.getRotation().getQuaternion().getY(),
              simulatedBallPose.getRotation().getQuaternion().getZ()
            });
        } else {
            // Eğer top havada değilse, belki de topu robotun içinde göstermek istersin
            // Veya tamamen ekrandan kaybetmek için boş bir array gönderebilirsin.
            SmartDashboard.putNumberArray("SimulatedBall", new double[] {
                0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0.0
            }); 
        }
}
}

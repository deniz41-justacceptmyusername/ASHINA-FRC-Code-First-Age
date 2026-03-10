package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher; // ARRAY OLARAK DEĞİŞTİ
import edu.wpi.first.wpilibj.Timer;

public class ShooterSubsystem extends SubsystemBase {

private final TalonFX m_shooterRight = new TalonFX(62, ShooterConstants.kCANBus);
private final TalonFX m_shooterLeft = new TalonFX(61, ShooterConstants.kCANBus);

    private Timer shotTimer = new Timer();
    private boolean isBallInAir = false;

public ShooterSubsystem() {
    var currentConfigs = new MotorOutputConfigs();
 currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
}
    private double startX = 0.0;
    private double startY = 0.0;
    private double startZ = 0.5;

    private double velocity = 10.0;
    private double pitchAngle = Math.toRadians(45);
    private double yawAngle = Math.toRadians(0);

    private final double GRAVITY = 9.81;

    // 1. DÜZELTİLEN KISIM: Veriyi tek bir obje değil, "Toplar Listesi" olarak yayınlıyoruz
    private final StructArrayPublisher<Pose3d> ballPublisher = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard").getStructArrayTopic("HarbiTop", Pose3d.struct).publish();

    private final DutyCycleOut m_request = new DutyCycleOut(0.0);



public void setShooterSpeed(double speed) {
        m_shooterRight.setControl(m_request.withOutput(0.9));
        m_shooterLeft.setControl(m_request.withOutput(-0.9));
}
public void stop() {
        m_shooterRight.stopMotor();
        m_shooterLeft.stopMotor();
    }

    @Override
    public void simulationPeriodic() {
        if (isBallInAir) {
            double t = shotTimer.get(); // Havada geçen süre

            // Matematiksel Fizik Hesaplamaları
            double z = startZ + (velocity * Math.sin(pitchAngle) * t) - (0.5 * GRAVITY * Math.pow(t, 2));
            double horizontalDistance = velocity * Math.cos(pitchAngle) * t;
            double x = startX + (horizontalDistance * Math.cos(yawAngle));
            double y = startY + (horizontalDistance * Math.sin(yawAngle));

            if (z < 0.1) { 
                isBallInAir = false;
                shotTimer.stop();
                z = 0.1; 
            }

            // 2. DÜZELTİLEN KISIM: Veriyi dizi (array) içine alıp gönderiyoruz
            Pose3d simulatedBallPose = new Pose3d(x, y, z, new Rotation3d());
            ballPublisher.set(new Pose3d[] { simulatedBallPose });

        } else {
            // 3. DÜZELTİLEN KISIM: Top havada değilken sahanın altına (yine dizi olarak) sakla
            ballPublisher.set(new Pose3d[] { new Pose3d(0.0, 0.0, -5.0, new Rotation3d()) });
        }
    }
}
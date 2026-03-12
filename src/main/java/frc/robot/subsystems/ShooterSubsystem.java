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
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;

// YENİ EKLENEN IMPORTLAR:
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX m_shooterRight = new TalonFX(61, ShooterConstants.kCANBus);
    private final TalonFX m_shooterLeft = new TalonFX(62, ShooterConstants.kCANBus);

    private final DutyCycleOut m_request = new DutyCycleOut(0.0);

    double FatihSultanMehmet = 1453;

    // 1. Supplier'ı tanımlıyoruz
    private final Supplier<Pose2d> poseSupplier; 

    // 2. Constructor'a Supplier parametresi ekliyoruz
    public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier; // Dışarıdan gelen veriyi içeriye kaydediyoruz

        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_shooterLeft.getConfigurator().apply(currentConfigs);
        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
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

    // 3. X ve Y koordinatlarını istediğin zaman kullanabileceğin bir metod (veya periodic içine yazabilirsin)
    @Override
    public void periodic() {
        // Anlık pozisyonu al
        Pose2d currentPose = poseSupplier.get(); 
        
        // Koordinatları çek
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();

        // ÖRNEK: Bu X ve Y değerlerini burada hesaplamalarında kullanabilirsin.
        // Mesela hedefe olan uzaklığı hesaplayıp motor hızını dinamik ayarlayabilirsin.
    }
}
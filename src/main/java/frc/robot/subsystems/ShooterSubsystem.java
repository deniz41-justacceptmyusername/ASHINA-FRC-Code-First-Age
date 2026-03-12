package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// YENİ EKLENEN IMPORTLAR:
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX m_shooterRight;// = new TalonFX(71, ShooterConstants.kCANBus);
    private final TalonFX m_shooterLeft;
    //private final TalonFX m_shooterLeft = new TalonFX(72, ShooterConstants.kCANBus);
    private final VelocityVoltage VelocityRequest;

    private final DutyCycleOut m_request = new DutyCycleOut(0.0);

    double FatihSultanMehmet = 1453;
    private final Supplier<Pose2d> poseSupplier; 
   public ShooterSubsystem(Supplier<Pose2d> poseSupplier) {
      this.poseSupplier = poseSupplier;
       m_shooterLeft = new TalonFX(72);
       m_shooterRight = new TalonFX(71);
        VelocityRequest = new VelocityVoltage(0).withSlot(0);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;

    var currentConfigs = new MotorOutputConfigs();
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_shooterLeft.getConfigurator().apply(currentConfigs);

        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        m_shooterRight.getConfigurator().apply(currentConfigs);
    }

    public void setShooterSpeed(double targetRPS) {
      m_shooterLeft.setControl(VelocityRequest.withVelocity(targetRPS));
      m_shooterRight.setControl(VelocityRequest.withVelocity(targetRPS));
        // m_shooterRight.setControl(m_request.withOutput(speed));
       // m_shooterLeft.setControl(m_request.withOutput(speed));
    }

    public double getCurrentRPS() {
        return m_shooterLeft.getVelocity().getValueAsDouble();

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
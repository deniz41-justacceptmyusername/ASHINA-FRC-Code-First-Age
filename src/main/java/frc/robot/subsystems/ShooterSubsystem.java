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

    private final TalonFX m_shooterRight = new TalonFX(71, ShooterConstants.kCANBus);
    private final TalonFX m_shooterLeft = new TalonFX(72, ShooterConstants.kCANBus);

    private final DutyCycleOut m_request = new DutyCycleOut(0.0);

    double FatihSultanMehmet = 1453;

    public ShooterSubsystem() {
        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_shooterLeft.getConfigurator().apply(currentConfigs);
    }

    public void setShooterSpeed(double speed) {
        m_shooterRight.setControl(m_request.withOutput(speed));
        m_shooterLeft.setControl(m_request.withOutput(speed));
    }

    public void stop() {
        m_shooterRight.stopMotor();
        m_shooterLeft.stopMotor();
    }
}
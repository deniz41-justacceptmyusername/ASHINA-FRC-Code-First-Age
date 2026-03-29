package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX m_shooterRight = new TalonFX(62, ShooterConstants.kCANBus);
    private final TalonFX m_shooterLeft = new TalonFX(61, ShooterConstants.kCANBus);
    private final PWMSparkMax m_shooterStopper = new PWMSparkMax(1);
    
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    
    private double m_targetRPS = 0.0;
    private final Timer m_timer = new Timer();
    
    private boolean m_isShootingActive = false; 

    public ShooterSubsystem() {
        var shooterConfigs = new MotorOutputConfigs();
        var pidConfigs = new Slot0Configs();

        shooterConfigs.Inverted = InvertedValue.Clockwise_Positive; 
        pidConfigs.kP = 0.11;
        pidConfigs.kV = 0.12;

        m_shooterRight.getConfigurator().apply(shooterConfigs);
        m_shooterRight.getConfigurator().apply(pidConfigs);

        shooterConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_shooterLeft.getConfigurator().apply(shooterConfigs);
        m_shooterLeft.getConfigurator().apply(pidConfigs);
    }

    public void setShooterVelocity(double rps) {
        m_targetRPS = Math.abs(rps); 
        
        if (!m_isShootingActive) {
            m_timer.restart(); 
            m_isShootingActive = true;
        }
    }

    @Override
    public void periodic() {
        if (m_isShootingActive && m_targetRPS > 0) {
            double elapsedTime = m_timer.get();

            if (elapsedTime < 0.2) {
                m_shooterStopper.set(-0.3); 
                
            } else {
                m_shooterRight.setControl(m_velocityRequest.withVelocity(65));
                m_shooterLeft.setControl(m_velocityRequest.withVelocity(65));

                double leftVelocity = Math.abs(m_shooterLeft.getVelocity().getValueAsDouble());
                double rightVelocity = Math.abs(m_shooterRight.getVelocity().getValueAsDouble());
                double shootingThreshold = 65 - 5;

                if (leftVelocity >= shootingThreshold && rightVelocity >= shootingThreshold) {
                    m_shooterStopper.set(0.3); // Fırlat!
                } else {
                    m_shooterStopper.set(-0.3); // Hala ivmeleniyor, stopper bekliyor
                }
            }
        } else {
            m_shooterStopper.stopMotor();
            m_shooterRight.setControl(m_velocityRequest.withVelocity(0));
            m_shooterLeft.setControl(m_velocityRequest.withVelocity(0));
        }
    }

    public void stop() {
        m_targetRPS = 0.0; 
        m_isShootingActive = false; 
        m_timer.stop();
        m_timer.reset();
        
        m_shooterRight.stopMotor();
        m_shooterLeft.stopMotor();
        m_shooterStopper.stopMotor();
    }

    @Override
    public void simulationPeriodic() {
    }
}
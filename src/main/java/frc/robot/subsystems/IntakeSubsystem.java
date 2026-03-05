// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Motorları Right ve Left olarak bağımsız tanımladık
  private final TalonFX m_intakefront = new TalonFX(51, IntakeConstants.kCANBus);
  private final TalonFX m_intakeback = new TalonFX(52, IntakeConstants.kCANBus);
  public int intakestat=1;
  // Shooter motorların duruyor
//  private final TalonFX m_shooterLeader = new TalonFX(61, IntakeConstants.kCANBus);
//  private final TalonFX m_shooterFollower = new TalonFX(62, IntakeConstants.kCANBus);

  // İki motoru aynı anda sürmek için ortak kontrol talebi
  private final DutyCycleOut m_request = new DutyCycleOut(0.0);

  public IntakeSubsystem() {
    var currentConfigs = new MotorOutputConfigs();
 currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
 // Motorun yapılandırma (configuration) nesnesini oluştur
var talonFXConfigs = new TalonFXConfiguration();

// Motor Çıktı ayarlarından NeutralMode'u Coast olarak seç
talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

// Ayarları Kraken motoruna uygula
      m_intakeback.getConfigurator().apply(currentConfigs);
    }
  
  // İki motoru aynı anda ve aynı hızda çalıştıran metot
  public void setIntakeSpeed(double speed) {
      if(intakestat==0){
        m_intakefront.setControl(m_request.withOutput(speed));}
      else m_intakefront.setControl(m_request.withOutput(0));
        //m_intakeLeft.setControl(m_request.withOutput(speed));
    }
    public void getdown() {
      m_intakefront.setControl(m_request.withOutput(0.1));
      intakestat = 0;
      m_intakeback.setNeutralMode(NeutralModeValue.Coast);
    }
    public void getup(){
      m_intakefront.setControl(m_request.withOutput(-0.1));
      intakestat = 1;
      m_intakeback.setNeutralMode(NeutralModeValue.Brake);
      frontstop();
    }

  // RB tuşundan elimizi çekince motorları durduracak metot
  public void frontstop() {
        m_intakefront.stopMotor();
        //m_intakeLeft.stopMotor();
  }
  public void backstop(){
    m_intakeback.stopMotor();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
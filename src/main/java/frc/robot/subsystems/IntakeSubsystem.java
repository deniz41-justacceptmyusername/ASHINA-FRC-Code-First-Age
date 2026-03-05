// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Motorları Right ve Left olarak bağımsız tanımladık
  private final TalonFX m_intakeRight = new TalonFX(51, IntakeConstants.kCANBus);
  private final TalonFX m_intakeLeft = new TalonFX(52, IntakeConstants.kCANBus);
  
  // Shooter motorların duruyor
//  private final TalonFX m_shooterLeader = new TalonFX(61, IntakeConstants.kCANBus);
//  private final TalonFX m_shooterFollower = new TalonFX(62, IntakeConstants.kCANBus);

  // İki motoru aynı anda sürmek için ortak kontrol talebi
  private final DutyCycleOut m_request = new DutyCycleOut(0.0);

  public IntakeSubsystem() {
    var currentConfigs = new MotorOutputConfigs();
 currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
      //m_intakeLeft.getConfigurator().apply(currentConfigs);
    }
  
  // İki motoru aynı anda ve aynı hızda çalıştıran metot
  public void setIntakeSpeed(double speed) {
        m_intakeRight.setControl(m_request.withOutput(speed));
        //m_intakeLeft.setControl(m_request.withOutput(speed));
    }

  // RB tuşundan elimizi çekince motorları durduracak metot
  public void stop() {
        m_intakeRight.stopMotor();
        //m_intakeLeft.stopMotor();
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
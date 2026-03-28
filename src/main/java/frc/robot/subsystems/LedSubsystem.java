package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    
    // Otonom ve Disabled sayaçları
    private int m_chaseOffset = 0;
    private int m_nefesSayaci = 0;
    private boolean m_nefesAliyor = true;

    // TELEOP SAYAÇLARI (Yeni Kayma ve Hız Kontrolü İçin)
    private int m_teleopOffset = 0;
    private int m_teleopSpeedCounter = 0;

    public LedSubsystem() {
        // LED Sinyal kablosu PWM 2'de
        m_led = new AddressableLED(2);
        
        // 36 LED
        m_ledBuffer = new AddressableLEDBuffer(36);
        m_led.setLength(m_ledBuffer.getLength());
        
        // Başlat
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        if (DriverStation.isAutonomousEnabled()) {
            otonomAnimasyonu(); 
        } else if (DriverStation.isTeleopEnabled()) {
            teleopAnimasyonu(); 
        } else {
            disabledAnimasyonu(); 
        }
        
        m_led.setData(m_ledBuffer);
    }

    // 🚀 OTONOM: Mavi-Sarı kayan yıldız efekti
    private void otonomAnimasyonu() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            if ((i + m_chaseOffset) % 10 == 0) {
                m_ledBuffer.setRGB(i, 0, 0, 255); // Mavi
            } else if ((i + m_chaseOffset) % 10 == 1) {
                m_ledBuffer.setRGB(i, 255, 100, 0); // Sarı/Turuncu kuyruk
            } else {
                m_ledBuffer.setRGB(i, 0, 0, 0); 
            }
        }
        
        m_chaseOffset += 1; 
        if (m_chaseOffset >= m_ledBuffer.getLength()) {
            m_chaseOffset = 0;
        }
    }

    // 🎮 TELEOP: Ortadan uçlara doğru kayarken sönen yavaş 3'lü Kırmızı
    private void teleopAnimasyonu() {
        // Önce ekranı temizle (Siyah)
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }

        // Başlangıç noktaları (Ortadaki 6 LED: Sol 15-16-17, Sağ 18-19-20)
        int solBaslangic = 15 - m_teleopOffset;
        int sagBaslangic = 18 + m_teleopOffset;

        // PARLAKLIK HESABI: 
        // Offset 0 (tam ortada) iken parlaklık 255. 
        // Her adımda parlaklık 14 birim azalır (Uçlara gelince 30'lara kadar düşer).
        int parlaklik = 255 - (m_teleopOffset * 14); 
        if (parlaklik < 0) parlaklik = 0; // Eksiye düşmemesi için güvenlik

        // SOL tarafa kayan 3 LED
        for(int i = 0; i < 3; i++) {
            int indeks = solBaslangic + i;
            if (indeks >= 0 && indeks < m_ledBuffer.getLength()) {
                m_ledBuffer.setRGB(indeks, parlaklik, 0, 0); // Kırmızı
            }
        }

        // SAĞ tarafa kayan 3 LED
        for(int i = 0; i < 3; i++) {
            int indeks = sagBaslangic + i;
            if (indeks >= 0 && indeks < m_ledBuffer.getLength()) {
                m_ledBuffer.setRGB(indeks, parlaklik, 0, 0); // Kırmızı
            }
        }

        // DAHA YAVAŞ HIZ KONTROLÜ
        m_teleopSpeedCounter++;
        if (m_teleopSpeedCounter >= 4) { // Bu sayıyı (4) artırırsan animasyon daha da yavaşlar
            m_teleopOffset++;
            m_teleopSpeedCounter = 0; // Sayacı sıfırla
        }

        // Gruplar uçlardan dışarı taştığında (offset 17'yi geçtiğinde) animasyonu başa sar
        if (m_teleopOffset > 17) {
            m_teleopOffset = 0;
        }
    }

    // 🛑 DISABLED: Nefes alan Kırmızı
    private void disabledAnimasyonu() {
        if (m_nefesAliyor) {
            m_nefesSayaci += 2;
            if (m_nefesSayaci >= 150) m_nefesAliyor = false;
        } else {
            m_nefesSayaci -= 2;
            if (m_nefesSayaci <= 0) m_nefesAliyor = true;
        }

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, m_nefesSayaci, 0, 0);
        }
    }
}
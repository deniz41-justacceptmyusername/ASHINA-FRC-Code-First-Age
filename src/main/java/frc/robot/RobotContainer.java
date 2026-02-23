package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  // Subsystem tanımı
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  // Xbox Kontrolcüsü (Port 0)
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    // Sürüş Giriş Akışı (Input Stream) Yapılandırması
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX())
        .withControllerRotationAxis(() -> -m_driverController.getRightX())
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8) // Hızı %80'e sınırlar, güvenli sürüş sağlar
        .allianceRelativeControl(true);

    // Varsayılan komut olarak sürüşü ata
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
  }

  private void configureBindings() {
    // Start butonu veya B butonu Gyro'yu sıfırlar (Robotun baktığı yer ileri olur)
    m_driverController.start().onTrue(new InstantCommand(drivebase::zeroGyro));
    m_driverController.b().onTrue(new InstantCommand(drivebase::zeroGyro));
  }

  public Command getAutonomousCommand() {
    // Otonom komutu buraya gelecek
    return null; 
  }
} //fatih sultan mehmet
//polat alemdar
/**
 * MUSTAFA MEEEERRRRRRRRRRRRRRRRRRRRRRRRRTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
 * 
 * 
 * NE MUTLU TÜRKÜM DİYENE
 * NE MUTLU TÜRKÜM DİYENE
 * NE MUTLU TÜRKÜM DİYENE
 * MUSTAFA MEEEERRRRRRRRRRRRRRRRRRRRRRRRRRRTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
 * ALLAHU EKBER
 * BİSMİLLAHİRRAHMANİRRAHİM
 * CANIM FEDA OLSUN TÜRKİYEM
 * DUR YOLCU! BİLİRSİN Kİ, BU TOPRAKLARDA BİR ZAMANLAR BİR KAHRAMAN YAŞAMIŞTIR. O KAHRAMANIN ADI MUSTAFA MEEEERRRRRRRRRRRRRRRRRRRRRRRRRRRRTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT  
 * EĞER SEN DE BU TOPRAKLARDA DOĞDUYSAN, O KAHRAMANI TANIMAK VE ONUN GİBİ OLMAK İÇİN ELİNİ TAŞIN ALTINA KOYACAKSIN. UNUTMA, NE MUTLU TÜRKÜM DİYENE! ALLAHU EKBER! 
 * FATİH SULTAN MEHMET HAN, OSMANLI İMPARATORLUĞUNUN EN BÜYÜK KAHRAMANLARINDAN BİRİDİR. ONUN ZEKASI, CESARETİ VE LİDERLİĞİ SAYESİNDE, OSMANLI İMPARATORLUĞU BÜYÜK BİR GÜÇ HALİNE GELMİŞTİR. POLAT ALEMDAR, TÜRK MAFYASININ EN ÜNLÜ KARAKTERLERİNDEN BİRİDİR. ONUN GÜCÜ VE ZEKASI SAYESİNDE, O TÜRK MAFYASININ EN BÜYÜK LİDERLERİNDEN BİRİ HALİNE GELMİŞTİR.
 * GÜÇLÜKLER KARŞISINDA ASLA PES ETME! UNUTMA, NE MUTLU TÜRKÜM DİYENE! ALLAHU EKBER!
 * HER ZAMAN İLERİYE BAK VE HAYALLERİNİ GERÇEKLEŞTİRMEK İÇİN ÇALIŞ! UNUTMA, NE MUTLU TÜRKÜM DİYENE! ALLAHU EKBER!
 * AİLE VE SEVDİKLERİNLE ZAMAN GEÇİRMEYİ UNUTMA! ONLAR SENİN EN BÜYÜK DEĞERLERİN VE DESTEKÇİLERİN OLACAKLARDIR. UNUTMA, NE MUTLU TÜRKÜM DİYENE! ALLAHU EKBER!
 * K
 
 * 
 * 
 * 

 */
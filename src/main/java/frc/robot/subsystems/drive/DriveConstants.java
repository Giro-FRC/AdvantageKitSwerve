package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.0; // Değişmesi gerekecek muhtemelen
  public static final double odometryFrequency = 100.0; // Hz

  public static final double trackWidth = 0.50; // Meters
  public static final double wheelBase = 0.70; // Meters

  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions (Phoneix
  // Tunerda ofset kısmına girdiğimiz değerlerin aynısı)
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // Device CAN IDs (Kod Pigeonda desteklediği için var. KUllanılmayacak.)
  public static final int pigeonCanId = 20;

  // (Motorları sıfırlayıp canleri buradakiler gibi olacak şekilde ayarlayın.)
  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 3;
  public static final int frontRightDriveCanId = 5;
  public static final int backRightDriveCanId = 7;

  public static final int frontLeftTurnCanId = 2;
  public static final int backLeftTurnCanId = 4;
  public static final int frontRightTurnCanId = 6;
  public static final int backRightTurnCanId = 8;

  public static final int frontLeftTurnCancoderId = 9;
  public static final int backLeftTurnCancoderId = 10;
  public static final int frontRightTurnCancoderId = 11;
  public static final int backRightTurnCancoderId = 12;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5); // Güncelle

  public static final double driveMotorReduction =
      ((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)); // Yaklaşık 8.14 yapıyor zaten

  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv = 0.1;

  public static final double driveSimP = 0.9;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.02317;
  public static final double driveSimKv = 0.07829;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 12.8;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotations
  // ->
  // Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM
  // ->
  // Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 2.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);
}

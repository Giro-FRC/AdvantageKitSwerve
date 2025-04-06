package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldPolygon {

  // Robotun hangi alanda olduğunu kontrol eden metod
  public int checkRobotPosition(Pose2d RobotPose) {
    double x = RobotPose.getX();
    double y = RobotPose.getY();

    if (x >= 4.501 && x <= 8.768 && y >= 6.176 && y <= 8.000) {
      // Sarı
      return 1;
    } else if (x >= 4.501 && x <= 8.768 && y >= 1.902 && y <= 6.176) {
      // Yeşil
      return 2;
    } else if (x >= 4.501 && x <= 8.768 && y >= 0 && y <= 1.902) {
      // Mavi
      return 3;
    } else if (x >= 0 && x <= 4.501 && y >= 0 && y <= 1.240) {
      // Pembe
      return 4;
    } else if (x >= 0 && x <= 4.501 && y >= 1.240 && y <= 6.760) {
      // Kırmızı
      return 5;
    } else if (x >= 0 && x <= 4.501 && y >= 6.760 && y <= 8.000) {
      // Turuncu
      return 6;
    } else {
      // Alanlarda Değil
      return 0;
    }
  }
}

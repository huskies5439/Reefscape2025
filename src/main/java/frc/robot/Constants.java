// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  //////// BasePilotable
  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  public static final double maxVitesseLineaire = 4.8;// Vitesse linéaire max du chassis //Valeur original 4.8 a note:à
  // ajuster
  public static final double maxVitesseRotation = Math.PI * 1.4; // radians per second //Originale REV = 2pi soirée 22
                                                                 // fevrier = 1.5

  public static final double maxVitesseModule = 4.46;// Vitesse maximale d'un module en m/s

  // Chassis configuration
  public static final double kTrackWidth = Units.inchesToMeters(25.5);

  // Distance between centers of right and left wheels on robot
  public static final double kWheelBase = Units.inchesToMeters(25.5);

  // Distance between front and back wheels on robot
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  public static final class Hauteur { // index 0 == hauteur ascenseur index 1 == angle poignet

    public static final double sol[] = { 0, 0 };
    public static final double L1[] = { 0.1, 0 };
    public static final double L2[] = { 0.125, 0 };
    public static final double L3[] = { 0.15, 0 };
    public static final double L4[] = { 0.2, 30 };

    public static final double algueBas[] = { 1, 30 };
    public static final double algueHaut[] = { 1.5, 30 };

    public static final double processeur[] = { 0.5, 0 };
    public static final double station[] = { 0.5, 45 };

    public static final double grimper[] = {0.5 , 0};

  }

  public static final class BoutonOperateur {

    // DriverStation commence a compter a partir de 1
    public static final int L1 = 1;
    public static final int L2 = 2;
    public static final int L3 = 3;
    public static final int L4 = 4;

    public static final int A = 5;
    public static final int B = 6;
    public static final int C = 7;
    public static final int D = 8;

    public static final int E = 9;
    public static final int F = 10;
    public static final int G = 11;
    public static final int H = 12;

    public static final int I = 13;
    public static final int J = 14;
    public static final int K = 15;
    public static final int L = 16;

  }

  public static class Branche {

    public static final Pose2d A = new Pose2d(3.153, 4.187, Rotation2d.fromDegrees(0));
    public static final Pose2d B = new Pose2d(3.153, 3.851, Rotation2d.fromDegrees(0));
    public static final Pose2d C = new Pose2d(3.680, 2.940, Rotation2d.fromDegrees(60));
    public static final Pose2d D = new Pose2d(3.992, 2.784, Rotation2d.fromDegrees(60));
    public static final Pose2d E = new Pose2d(4.987, 2.784, Rotation2d.fromDegrees(120));
    public static final Pose2d F = new Pose2d(5.311, 2.940, Rotation2d.fromDegrees(120));
    public static final Pose2d G = new Pose2d(5.815, 3.860, Rotation2d.fromDegrees(-180));
    public static final Pose2d H = new Pose2d(5.815, 4.196, Rotation2d.fromDegrees(-180));
    public static final Pose2d I = new Pose2d(5.275, 5.058, Rotation2d.fromDegrees(-120));
    public static final Pose2d J = new Pose2d(5.021, 5.244, Rotation2d.fromDegrees(-120));
    public static final Pose2d K = new Pose2d(3.978, 5.224, Rotation2d.fromDegrees(-60));
    public static final Pose2d L = new Pose2d(3.715, 5.107, Rotation2d.fromDegrees(-60));
  }

  public static class Algue {

    public static final Pose2d AB = new Pose2d(3.188, 4.035, Rotation2d.fromDegrees(0));
    public static final Pose2d CD = new Pose2d(3.861, 2.855, Rotation2d.fromDegrees(60));
    public static final Pose2d EF = new Pose2d(5.119, 2.894, Rotation2d.fromDegrees(120));
    public static final Pose2d GH = new Pose2d(5.811, 4.015, Rotation2d.fromDegrees(-180));
    public static final Pose2d IJ = new Pose2d(5.168, 5.146, Rotation2d.fromDegrees(-120));
    public static final Pose2d KL = new Pose2d(3.832, 5.107, Rotation2d.fromDegrees(-60));

  }

  public static class GamePositions {
    //Terrain x = 17.5 y = 8 valeur à ajuster
    public static final Pose2d BlueCoralStationProc = new Pose2d(1.6, 0.7, Rotation2d.fromDegrees(-126));
    public static final Pose2d BlueCoralStationCage = new Pose2d(1.6, 7.4, Rotation2d.fromDegrees(126));
    public static final Pose2d BlueProcesseur = new Pose2d(6.0, 0.55, Rotation2d.fromDegrees(-90));
    public static final Pose2d BlueCentreRecif = new Pose2d(4.5,4,Rotation2d.kZero);
    // position bleu/rouge pour IsProche
    public static final Pose2d RedCoralStationProc = new Pose2d(15.9, 7.4, Rotation2d.fromDegrees(54));
    public static final Pose2d RedCoralStationCage = new Pose2d(15.9, 0.7, Rotation2d.fromDegrees(-54));
    public static final Pose2d RedProcesseur = new Pose2d(11.5, 7.45, Rotation2d.fromDegrees(90));
    public static final Pose2d RedCentreRecif = new Pose2d(13,4,Rotation2d.kZero);
  }

  public static final double kG = 0.3;

}

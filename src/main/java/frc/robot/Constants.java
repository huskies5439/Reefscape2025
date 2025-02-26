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

    public static final double sol[] = { 0, 90 };
    public static final double L1[] = { 0, 60};
    public static final double L2[] = { 0.123, -86 };
    public static final double L3[] = { 0.32, -86 };
    public static final double L4[] = { 0.56, -30 };

    public static final double algueSol[] = { 0, 7 };
    public static final double algueBas[] = { 0.38, -19 };
    public static final double algueHaut[] = { 0.537, -12 };

    public static final double processeur[] = { 0.08, 0 };
    public static final double station[] = { 0.01, 83 };

    public static final double grimper[] = {0.13, 0};

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

    public static final Pose2d A = new Pose2d(3.203, 4.177, Rotation2d.fromDegrees(0));
    public static final Pose2d B = new Pose2d(3.153, 3.851, Rotation2d.fromDegrees(0));
    public static final Pose2d C = new Pose2d(3.720, 2.990, Rotation2d.fromDegrees(60));
    public static final Pose2d D = new Pose2d(3.992, 2.784, Rotation2d.fromDegrees(60));
    public static final Pose2d E = new Pose2d(4.997, 2.854, Rotation2d.fromDegrees(120));
    public static final Pose2d F = new Pose2d(5.281, 3.01, Rotation2d.fromDegrees(120));
    public static final Pose2d G = new Pose2d(5.795, 3.860, Rotation2d.fromDegrees(-180));
    public static final Pose2d H = new Pose2d(5.795, 4.196, Rotation2d.fromDegrees(-180));
    public static final Pose2d I = new Pose2d(5.235, 5.078, Rotation2d.fromDegrees(-120));
    public static final Pose2d J = new Pose2d(4.941, 5.214, Rotation2d.fromDegrees(-120));
    public static final Pose2d K = new Pose2d(3.978, 5.204, Rotation2d.fromDegrees(-60));
    public static final Pose2d L = new Pose2d(3.675, 5.067, Rotation2d.fromDegrees(-60));
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
    //Proche/Centre/Loin selon la driver station
    //deltaX = 0.45 et deltaY = -0.33
    public static final Pose2d BlueCoralStationProcProche = new Pose2d(0.53, 1.284, Rotation2d.fromDegrees(-126));
    public static final Pose2d BlueCoralStationProcCentre = new Pose2d(0.98, 0.954, Rotation2d.fromDegrees(-126));
    public static final Pose2d BlueCoralStationProcLoin = new Pose2d(1.43, 0.624, Rotation2d.fromDegrees(-126));

    //Calcul par rapport à Processeur : même x, yProc + yCage = 8 m
    public static final Pose2d BlueCoralStationCageProche = new Pose2d(0.53, 6.716, Rotation2d.fromDegrees(126));
    public static final Pose2d BlueCoralStationCageCentre = new Pose2d(0.98, 7.046, Rotation2d.fromDegrees(126));
    public static final Pose2d BlueCoralStationCageLoin = new Pose2d(1.43, 7.376, Rotation2d.fromDegrees(126));


    public static final Pose2d BlueProcesseur = new Pose2d(6.0, 0.55, Rotation2d.fromDegrees(-90));
    public static final Pose2d BlueCentreRecif = new Pose2d(4.5,4,Rotation2d.kZero);

    // Positions rouges seulement nécessaires pour les fonctions isProche, sinon on n'utilise que les Blue pour le Pathfinding
    public static final Pose2d RedCoralStationProc = new Pose2d(16.39, 7.077, Rotation2d.fromDegrees(54));//Ajuster pour être au CENTRE de la coral station côté rouge
    public static final Pose2d RedCoralStationCage = new Pose2d(16.39, 0.993, Rotation2d.fromDegrees(-54));//IDEM
    public static final Pose2d RedProcesseur = new Pose2d(11.5, 7.45, Rotation2d.fromDegrees(90));
    public static final Pose2d RedCentreRecif = new Pose2d(13,4,Rotation2d.kZero);
  }
}

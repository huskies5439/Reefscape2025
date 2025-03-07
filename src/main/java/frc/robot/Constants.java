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
  public static final double maxVitesseLineaire = 3.75;// Vitesse linéaire max du chassis 

  public static final double maxVitesseRotation = Math.PI * 1.4; // radians per second

  // public static final double maxVitesseModule = 4.46;// Vitesse maximale d'un module en m/s, pas nécessaire si setpoint generator

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
    public static final double L2[] = { 0.122, -85 };
    public static final double L3[] = { 0.250, -20 };
    public static final double L4[] = { 0.56, -30 };

    public static final double algueSol[] = { 0, 0 };
    public static final double algueBas[] = { 0.378, -32 };
    public static final double algueHaut[] = { 0.555, -29 };

    public static final double processeur[] = { 0.08, 0 };
    public static final double station[] = { 0.01, 80 };

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

    public static final Pose2d A = new Pose2d(3.26, 4.25, Rotation2d.fromDegrees(0));
    public static final Pose2d B = new Pose2d(3.26, 3.92, Rotation2d.fromDegrees(0));
    public static final Pose2d C = new Pose2d(3.68, 3.00, Rotation2d.fromDegrees(60));
    public static final Pose2d D = new Pose2d(3.97, 2.84, Rotation2d.fromDegrees(60));
    public static final Pose2d E = new Pose2d(5.00, 2.85, Rotation2d.fromDegrees(120));
    public static final Pose2d F = new Pose2d(5.25, 3.00, Rotation2d.fromDegrees(120));
    public static final Pose2d G = new Pose2d(5.79, 3.88, Rotation2d.fromDegrees(-180));
    public static final Pose2d H = new Pose2d(5.79, 4.21, Rotation2d.fromDegrees(-180));
    public static final Pose2d I = new Pose2d(5.27, 5.08, Rotation2d.fromDegrees(-120));
    public static final Pose2d J = new Pose2d(4.97, 5.22, Rotation2d.fromDegrees(-120));
    public static final Pose2d K = new Pose2d(3.95, 5.25, Rotation2d.fromDegrees(-60));
    public static final Pose2d L = new Pose2d(3.70, 5.05, Rotation2d.fromDegrees(-60));
  }

  public static class Algue {
    public static final Pose2d AB = new Pose2d(3.26, 4.09, Rotation2d.fromDegrees(0));
    public static final Pose2d CD = new Pose2d(3.82, 2.92, Rotation2d.fromDegrees(60));
    public static final Pose2d EF = new Pose2d(5.13, 2.93, Rotation2d.fromDegrees(120));
    public static final Pose2d GH = new Pose2d(5.79, 4.05, Rotation2d.fromDegrees(-180));
    public static final Pose2d IJ = new Pose2d(5.12, 5.15, Rotation2d.fromDegrees(-120));
    public static final Pose2d KL = new Pose2d(3.83, 5.15, Rotation2d.fromDegrees(-60));

  }

  public static class GamePositions {
    
    public static final Pose2d BlueCoralStationProcProche = new Pose2d(0.61, 1.34, Rotation2d.fromDegrees(-126));
    public static final Pose2d BlueCoralStationProcCentre = new Pose2d(1.07, 0.99, Rotation2d.fromDegrees(-126));
    public static final Pose2d BlueCoralStationProcLoin = new Pose2d(1.53, 0.64, Rotation2d.fromDegrees(-126));

    public static final Pose2d BlueCoralStationCageProche = new Pose2d(0.65, 6.75, Rotation2d.fromDegrees(126));
    public static final Pose2d BlueCoralStationCageCentre = new Pose2d(1.11, 7.09, Rotation2d.fromDegrees(126));
    public static final Pose2d BlueCoralStationCageLoin = new Pose2d(1.57, 7.43, Rotation2d.fromDegrees(126));


    public static final Pose2d BlueProcesseur = new Pose2d(6.00, 0.70, Rotation2d.fromDegrees(-90));
    public static final Pose2d BlueCentreRecif = new Pose2d(4.5,4,Rotation2d.kZero);

    // Positions rouges seulement nécessaires pour les fonctions isProche, sinon on n'utilise que les Blue pour le Pathfinding
    public static final Pose2d RedCoralStationProc = new Pose2d(16.39, 7.077, Rotation2d.fromDegrees(54));//Ajuster pour être au CENTRE de la coral station côté rouge
    public static final Pose2d RedCoralStationCage = new Pose2d(16.39, 0.993, Rotation2d.fromDegrees(-54));//IDEM
    public static final Pose2d RedProcesseur = new Pose2d(11.5, 7.45, Rotation2d.fromDegrees(90));
    public static final Pose2d RedCentreRecif = new Pose2d(13,4,Rotation2d.kZero);
  }
}

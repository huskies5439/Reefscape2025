// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Algue;
import frc.robot.Constants.BoutonOperateur;
import frc.robot.Constants.Branche;
import frc.robot.Constants.Hauteur;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;


public class RamasserAlgue extends ParallelCommandGroup {
  private Pose2d cibleManette;
  private double cibleHauteurAlgue[];
  private Pose2d ciblePositionAlgue;

  public RamasserAlgue(Ascenseur ascenseur, Poignet poignet, BasePilotable basePilotable, AlgueManip algueManip) {

    cibleManette = basePilotable.getCibleRecif();

    if (cibleManette == Branche.A || cibleManette == Branche.B) {
      ciblePositionAlgue = Algue.AB;
      cibleHauteurAlgue = Hauteur.algueHaut;

    } else if (cibleManette == Branche.C || cibleManette == Branche.D) {
      ciblePositionAlgue = Algue.CD;
      cibleHauteurAlgue = Hauteur.algueBas;

    } else if (cibleManette == Branche.E || cibleManette == Branche.F) {
      ciblePositionAlgue = Algue.EF;
      cibleHauteurAlgue = Hauteur.algueHaut;

    } else if (cibleManette == Branche.G || cibleManette == Branche.H) {
      ciblePositionAlgue = Algue.GH;
      cibleHauteurAlgue = Hauteur.algueBas;

    } else if (cibleManette == Branche.I || cibleManette == Branche.J) {
      ciblePositionAlgue = Algue.IJ;
      cibleHauteurAlgue = Hauteur.algueHaut;

    } else if (cibleManette == Branche.K || cibleManette == Branche.L) {
      ciblePositionAlgue = Algue.KL;
      cibleHauteurAlgue = Hauteur.algueBas;

    }

    addCommands(

        basePilotable.followPath(ciblePositionAlgue),
        new SequentialCommandGroup(
            new WaitUntilCommand(() -> basePilotable.isProche(ciblePositionAlgue, Constants.distanceMin)),
            new GoToHauteur(cibleHauteurAlgue[0], cibleHauteurAlgue[1], ascenseur, poignet))

    );
  }
}

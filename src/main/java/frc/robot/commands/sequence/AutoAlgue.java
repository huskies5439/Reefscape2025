// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Algue;
import frc.robot.Constants.Branche;
import frc.robot.Constants.Hauteur;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

public class AutoAlgue extends ParallelCommandGroup {
  private Pose2d cibleManette;
  private double cibleHauteurAlgue[];
  private Pose2d ciblePositionAlgue;

  public AutoAlgue(Pose2d cible, double[] hauteur, Ascenseur ascenseur, Poignet poignet, BasePilotable basePilotable, AlgueManip algueManip) {
    // compile les infos de la manette operateur
    //  Détermine automatiquement quel algue il faut gober
    //!!NE GOBE PAS AUTOMATIQUEMENT!!
    cibleManette = basePilotable.getCibleManetteOperateur();


    addCommands(
        // se rend automatiquement à la bonne position sur le recif et se rend a la bonne hauteur 
        basePilotable.followPath(ciblePositionAlgue),
        new SequentialCommandGroup(
            new WaitUntilCommand(basePilotable::isProcheRecif),
            new GoToHauteur(()-> cibleHauteurAlgue[0], ()-> cibleHauteurAlgue[1], ascenseur, poignet)
            )

    );
  }
}

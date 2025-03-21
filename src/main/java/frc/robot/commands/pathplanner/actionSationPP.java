// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pathplanner;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.Hauteur;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.CorailManip;
import frc.robot.subsystems.Poignet;

public class actionSationPP extends ParallelRaceGroup {

  //Actions durant le déplacement vers la station
  public actionSationPP(BasePilotable basePilotable, Ascenseur ascenseur, Poignet poignet, CorailManip corailManip) {

    addCommands(
      new GoToHauteur(()-> Hauteur.station[0], ()-> Hauteur.station[1], ascenseur, poignet),//GoToHauteur nécessaire car le trigger automatique n'est pas géré par PathPlanner en autonome
      corailManip.goberCommand().until(corailManip::isCorail)
        );
  }
}

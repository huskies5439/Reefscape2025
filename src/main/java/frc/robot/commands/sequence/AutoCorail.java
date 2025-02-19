// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

// Une fonction de téléop qui se termine quand le pilote lache le piton
//!!NE LIVRE PAS LE CORAIL!!
public class AutoCorail extends ParallelCommandGroup {
  //compile les infos de la manette operateur
  // se rend automatiquement à la bonne position sur le recif
  public AutoCorail(BasePilotable basePilotable, Ascenseur ascenseur, Poignet poignet) {
    Pose2d cible = basePilotable.getCibleManetteOperateur();
    addCommands(
      basePilotable.followPath(cible),
      
      //quand suffisament proche, met l'ascenseur et le poignet à la bonne position
      new SequentialCommandGroup(
        //new WaitUntilCommand(basePilotable::isProcheRecif),
        //new GoToHauteur(()-> ascenseur.getCibleManetteOperateur(), ()-> poignet.getCibleManetteOperateur(), ascenseur, poignet)
      )
    );
  }
}

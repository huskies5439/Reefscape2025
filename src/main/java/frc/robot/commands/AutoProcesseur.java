// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.GamePositions;
import frc.robot.Constants.Hauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;


public class AutoProcesseur extends ParallelCommandGroup {
  //vas automatiquement au prosseseur
  //!!NE LIVRE PAS L'ALGUE!!
  public AutoProcesseur(BasePilotable basePilotable, Ascenseur ascenseur, Poignet poignet) {
   Pose2d cible = GamePositions.Processeur;
    addCommands(
       basePilotable.followPath(cible),
      
      //faut t'il vraiment attendre?
      new SequentialCommandGroup(
        new WaitUntilCommand(()-> basePilotable.isProche(cible, Constants.distanceMin)),
        new GoToHauteur(Hauteur.processeur[0], Hauteur.processeur[1], ascenseur, poignet)
      )
    );
  }
}

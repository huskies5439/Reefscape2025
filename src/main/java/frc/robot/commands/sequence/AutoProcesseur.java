// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.GamePositions;
import frc.robot.Constants.Hauteur;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;


public class AutoProcesseur extends ParallelCommandGroup {
  //vas automatiquement au prosseseur
  //!!NE LIVRE PAS L'ALGUE!!
  public AutoProcesseur(BasePilotable basePilotable, Ascenseur ascenseur, Poignet poignet) {
    addCommands(
      basePilotable.followPath(GamePositions.BlueProcesseur),
      
      new SequentialCommandGroup(
         new WaitUntilCommand(basePilotable::isProcheProcesseur),
         new GoToHauteur(()-> Hauteur.processeur[0], ()-> Hauteur.processeur[1], ascenseur, poignet)
      )
    );
  }
}

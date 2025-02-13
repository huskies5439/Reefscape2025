// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pathplanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Hauteur;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;


public class ActionRecifAlgueBasPathPlanner extends SequentialCommandGroup {

  /**Actions durant le déplacement vers le récif pour une algue basse*/
  public ActionRecifAlgueBasPathPlanner(BasePilotable basePilotable, Ascenseur ascenseur, Poignet poignet) {

    addCommands( new GoToHauteur(Hauteur.sol[0], Hauteur.sol[1], ascenseur, poignet),
        new WaitUntilCommand(basePilotable::isProcheRecif),
        new GoToHauteur(Hauteur.algueBas[0], Hauteur.algueBas[1], ascenseur, poignet));
  }
}

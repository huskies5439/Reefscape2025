// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pathplanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

public class actionCorailPP extends SequentialCommandGroup {

  /** Actions durant le déplacement vers le récif pour un corail */
  public actionCorailPP(double[] hauteur, BasePilotable basePilotable, Ascenseur ascenseur, Poignet poignet) {

    addCommands(
        
        new WaitUntilCommand(basePilotable::isProcheRecif),
        new GoToHauteur(()-> hauteur[0], ()-> hauteur[1], ascenseur, poignet)
        .until(()-> {return ascenseur.atCible() && poignet.atCible();})
        );
  }
}

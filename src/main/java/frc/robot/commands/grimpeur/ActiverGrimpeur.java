// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grimpeur;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Hauteur;
import frc.robot.commands.GoToHauteurAvecFin;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;

public class ActiverGrimpeur extends SequentialCommandGroup {
  
  //Commande qui initialise le mode Grimpeur
  public ActiverGrimpeur(Ascenseur ascenseur, Poignet poignet) { 
    addCommands(
        new InstantCommand(ascenseur::debarrer, ascenseur),//Normalement le servo n'est pas barré, c'est une sécurité
        new GoToHauteurAvecFin(()-> Hauteur.grimper[0], ()-> Hauteur.grimper[1], ascenseur, poignet)//Il faut une fin pour passer au contrôle du mode grimpeur
          .raceWith(new WaitCommand(2))//Sécurité au cas où la batterie soit dead et qu'on atteigne pas les cibles
        );
  }
}

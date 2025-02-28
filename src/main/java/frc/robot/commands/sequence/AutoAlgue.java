// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

public class AutoAlgue extends ParallelRaceGroup {
  //Aller chercher les Algues automatiquement au Récif
  //Cette commande doit reculer avant de descendre l'ascenseur à cause de a taille de l'Algue
  //Contrairement aux autres AutoXYZ, se termine automatiquement.
  //Si on ne veut pas que ça se termine automatiquement, ôter la Race, mais l'ascenseur va rester dans les airs
  public AutoAlgue(Pose2d cible, double[] hauteur, Ascenseur ascenseur, Poignet poignet, BasePilotable basePilotable, AlgueManip algueManip,  CommandXboxController manette) {

    addCommands(
        new SequentialCommandGroup(
          basePilotable.followPath(cible)//Se rendre au récif
            .until(algueManip::isAlgue),//Qu'on se rende ou non, continuer quand on clique l'algue
        
            new ConditionalCommand(
          Commands.run(
          () -> basePilotable.conduire(
                0.5*manette.getLeftY(), 0.5*manette.getLeftX(),
                0.5*manette.getRightX(),
                  true, true),
          basePilotable),
          new WaitCommand(0.01), 
          ()-> {return DriverStation.isTeleop();})
       
         
          
        ),
    
        //Monter quand on approche du récif
        //On gobe en même temps
        new SequentialCommandGroup(
             new WaitUntilCommand(basePilotable::isProcheRecif),
             new GoToHauteur(()-> hauteur[0], ()-> hauteur[1], ascenseur, poignet).alongWith(algueManip.goberCommand())
             )

    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

public class AutoAlgue extends ParallelCommandGroup {
  //Aller chercher les Algues automatiquement au Récif
  //Cette commande doit reculer avant de descendre l'ascenseur à cause de a taille de l'Algue
  //Contrairement aux autres AutoXYZ, se termine automatiquement.
  //Si on ne veut pas que ça se termine automatiquement, ôter la Race, mais l'ascenseur va rester dans les airs
  public AutoAlgue(Pose2d cible, double[] hauteur, Ascenseur ascenseur, Poignet poignet, BasePilotable basePilotable, AlgueManip algueManip) {

    addCommands(

      basePilotable.followPath(cible),//Se rendre au récif
      //Qu'on se rende ou non, continuer quand on clique l'algue
    
        //Monter quand on approche du récif
        //On gobe en même temps
        new SequentialCommandGroup(
             new WaitUntilCommand(basePilotable::isProcheRecif),
             new GoToHauteur(()-> hauteur[0], ()-> hauteur[1], ascenseur, poignet).alongWith(algueManip.goberCommand())
             )

    );
  }
}

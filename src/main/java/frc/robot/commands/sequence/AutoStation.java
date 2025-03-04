// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.CorailManip;
import frc.robot.subsystems.Poignet;


public class AutoStation extends SequentialCommandGroup {
  //Contrairement aux autres AutoXYZ, l'ascenceur est géré automatiquement par un trigger arbitraire.
  //La Commande ne fait que se rendre à la station.
  public AutoStation(Pose2d cible, BasePilotable basePilotable,Ascenseur ascenseur, Poignet poignet, CorailManip corailManip ) {
   //se rend automatiquement à la station de Corail

    addCommands(
      basePilotable.followPath(cible),
      Commands.run(basePilotable::setX, basePilotable)//On barre les roues.car la station n'es pas protégée
      );
  }
}

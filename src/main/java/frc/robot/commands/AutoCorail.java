// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

// Une fonction de téléop qui se termine quand le pilote lache le piton
public class AutoCorail extends ParallelCommandGroup {
  /** Creates a new AutoCorail. */
  public AutoCorail(BasePilotable basePilotable, Ascenseur ascenseur, Poignet poignet) {
    Pose2d cible = basePilotable.getCibleRecif();
    addCommands(
      basePilotable.followPath(cible),
      
      new SequentialCommandGroup(
        new WaitUntilCommand(()-> basePilotable.isProche(cible, Constants.distanceMin)),
        new GoToHauteur(ascenseur.getCibleRecif(), poignet.getCibleRecif(), ascenseur, poignet)
      )
    );
  }
}

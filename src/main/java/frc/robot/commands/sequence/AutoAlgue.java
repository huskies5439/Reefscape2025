// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

public class AutoAlgue extends ParallelCommandGroup {

  public AutoAlgue(Pose2d cible, double[] hauteur, Ascenseur ascenseur, Poignet poignet, BasePilotable basePilotable, AlgueManip algueManip) {

    addCommands(
        // se rend automatiquement à la bonne position sur le recif et se rend a la bonne hauteur 
        // basePilotable.followPath(cible),
        // new SequentialCommandGroup(
        //     new WaitUntilCommand(basePilotable::isProcheRecif),
             new GoToHauteur(()-> hauteur[0], ()-> hauteur[1], ascenseur, poignet).alongWith(algueManip.goberCommand())
        //     )

    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequence;

import java.io.SequenceInputStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GoToHauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

// Une fonction de téléop qui se termine quand le pilote lache le piton
//!!NE LIVRE PAS LE CORAIL!!
public class AutoCorail extends ParallelCommandGroup {
  // se rend automatiquement à la bonne position sur le recif
  public AutoCorail(Pose2d cible, BasePilotable basePilotable, Ascenseur ascenseur, Poignet poignet, CommandXboxController manette) {
    addCommands(

      basePilotable.followPath(cible),
      
      //quand suffisament proche, met l'ascenseur et le poignet à la bonne position
      new SequentialCommandGroup(
        new WaitUntilCommand(basePilotable::isProcheRecif),
        new GoToHauteur(ascenseur::getCibleManetteOperateur,
						poignet::getCibleManetteOperateur, ascenseur, poignet)
      )
    );

  }
}

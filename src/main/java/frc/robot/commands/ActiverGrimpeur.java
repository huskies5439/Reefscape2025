// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Hauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;


public class ActiverGrimpeur extends SequentialCommandGroup {

  public ActiverGrimpeur(Ascenseur ascenseur, Poignet poignet ) {   
    addCommands(
      new GoToHauteur(Hauteur.grimper[0], Hauteur.grimper[1], ascenseur, poignet ),
      new InstantCommand(ascenseur :: debarrer, ascenseur)
      );
  }
}

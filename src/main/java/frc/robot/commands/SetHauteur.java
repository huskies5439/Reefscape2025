// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;

public class SetHauteur extends ParallelCommandGroup {
  public SetHauteur(double[] cible, Ascenseur ascenseur, Poignet poignet) {

    addCommands(
        Commands.runOnce(() -> ascenseur.setHauteurCible(cible[0])),
        Commands.runOnce(() -> poignet.setAngleCible(cible[1])));
  }
}

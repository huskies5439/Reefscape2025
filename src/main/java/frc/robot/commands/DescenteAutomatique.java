// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Poignet;

public class DescenteAutomatique extends Command {
  BasePilotable basePilotable;
  Ascenseur ascenseur;
  Poignet poignet;


  //Sert à laisser la pince dans les airs quand on est proche du récif.
  //Overide la commande par défaut qui fait descendre automatiquement

  //Avec le recul (à une journée de la compé), ce code aurait 100% pu être intégré dans PoignetDefaut et AscenceurDefaut......
  public DescenteAutomatique(BasePilotable basePilotable, Ascenseur ascenseur, Poignet poignet) {
    this.basePilotable = basePilotable;
    this.ascenseur = ascenseur;
    this.poignet = poignet;

    addRequirements(ascenseur, poignet);//On ne demande explicitement pas la basepilotable car on veut pouvoir conduire !!
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    ascenseur.hold();
    poignet.hold();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return basePilotable.isLoinRecif();
  }
}

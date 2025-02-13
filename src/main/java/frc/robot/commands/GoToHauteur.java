// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;

public class GoToHauteur extends Command {
  private double cibleAscenceur;
  private double ciblePoignet;
  private Ascenseur ascenseur;
  private Poignet poignet;


  //Double PID pour le poignet et l'ascenseur
  //que faire après la command? tel est la question
  public GoToHauteur(double cibleAscenceur, double ciblePoignet, Ascenseur ascenseur, Poignet poignet) {
    this.cibleAscenceur = cibleAscenceur;
    this.ciblePoignet = ciblePoignet;
    this.ascenseur = ascenseur;
    this.poignet = poignet;
    addRequirements(ascenseur, poignet);
  }

  @Override
  public void initialize() {
    ascenseur.resetPID();
    poignet.resetPID();
  }

  @Override
  public void execute() {
    ascenseur.setPID(cibleAscenceur);
    poignet.setPID(ciblePoignet);

  }

  @Override
  public void end(boolean interrupted) {
    ascenseur.hold();
    poignet.hold();
  }

  @Override
  public boolean isFinished() {
    return ascenseur.atCible() && poignet.atCible();
  }
}

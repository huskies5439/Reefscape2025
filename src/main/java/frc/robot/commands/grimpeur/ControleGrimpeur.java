// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.grimpeur;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascenseur;

public class ControleGrimpeur extends Command {

  private Ascenseur ascenseur;
  private double voltageDemande;

  private DoubleSupplier monter;
  private DoubleSupplier descendre;


  //Descendre et monter l'ascenseur dans le mode grimpeur
  public ControleGrimpeur(DoubleSupplier monter, DoubleSupplier descendre, Ascenseur ascenseur) {
    this.ascenseur = ascenseur;
    this.monter = monter;
    this.descendre = descendre;
    this.voltageDemande = 0;
  }

  @Override
  public void initialize() {
    ascenseur.debarrer();//Normalement le servo est débarré, c'est une double sécurité
  }

  @Override
  public void execute() {
    voltageDemande = monter.getAsDouble() - descendre.getAsDouble();//Classique mouvement avec les deux triggers de la manette

    if (voltageDemande == 0) {
      if (!ascenseur.isLimitSwitch()) {
        /* Le cas de figure ici c'est quand on est sur les limit switches, donc l'ascenseur sous le cadre périphérique,
         * donc dans les airs (!), on ne veut pas hold car on veut les moteurs désactivés quand on barre avec le servo.
         * Par contre, si on est dans les airs sans appuyé sur les triggers, on veut hold pour ne pas que la pince descende
         * durant le recul vers la cage. */
        ascenseur.hold();
      }
    } else {
      ascenseur.setVoltage(voltageDemande * 6);
    }

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

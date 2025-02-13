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

  public ControleGrimpeur(DoubleSupplier monter, DoubleSupplier descendre, Ascenseur ascenseur) {
    this.ascenseur = ascenseur;
    this.monter = monter;
    this.descendre = descendre;
    this.voltageDemande = 0;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    voltageDemande = monter.getAsDouble() - descendre.getAsDouble();

    if (ascenseur.isLimitSwitch() && voltageDemande == 0) {
      ascenseur.barrer();
    } else {
      ascenseur.debarrer();
    }
    ascenseur.setVoltage(voltageDemande * 3); // à ajuster
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      ascenseur.debarrer();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

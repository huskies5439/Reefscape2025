// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Hauteur;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Poignet;

public class PoignetDefaut extends Command {
  Poignet poignet;
  AlgueManip algueManip;

  boolean modeManuel;
  BooleanSupplier monterBouton;
  BooleanSupplier descendreBouton;
  boolean monter;
  boolean descendre;

  // Remonte automatique le poignet
  // Passe en mode manuel si le dPad est utilisé
  public PoignetDefaut(BooleanSupplier monterBouton, BooleanSupplier descendreBouton, Poignet poignet, AlgueManip algueManip) {
    this.poignet = poignet;
    this.algueManip = algueManip;

    this.monterBouton = monterBouton;
    this.descendreBouton = descendreBouton;
    modeManuel = false;

    addRequirements(poignet);
  }

  @Override
  public void initialize() {
    modeManuel = false;
  }

  @Override
  public void execute() {
    // Sauvegarde les valeurs des boutons monter et descendre
    monter = monterBouton.getAsBoolean();
    descendre = descendreBouton.getAsBoolean();

    // Appuyer sur le dPad = mode manuel
    if (monter || descendre) {
      modeManuel = true;
    }

    if (!modeManuel) {//PID si on n'a pas touché au dPad
      if (poignet.getAngle() < 85) {// Désactive le PID si on est proche de la verticale
        if (algueManip.isAlgue()) {// Si on a une algue, on va plutôt à l'horizontale
          poignet.setPID(0);
        } else {
          poignet.setPID(Hauteur.sol[1]);
        }
      } else {
        poignet.stop();
      }
    }

    //sinon, dPad jusqu'à ce que la fonction soit caller à nouveau
    else{
      if(monter){
        poignet.monter();
      }
      else if(descendre){
        poignet.descendre();
      }
      else{
        poignet.hold();
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
    poignet.stop();
    modeManuel = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

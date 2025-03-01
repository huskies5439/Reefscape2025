// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Hauteur;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;

public class AscenseurDefaut extends Command {

  Ascenseur ascenseur;
  Poignet poignet;
  
  boolean modeManuel;
  BooleanSupplier monterBouton;
  BooleanSupplier descendreBouton;
  boolean monter;
  boolean descendre;

  // Descend l'ascenseur automatique au sol
  // Passe en mode manuel si le dPad est utilisé
  public AscenseurDefaut(BooleanSupplier monterBouton, BooleanSupplier descendreBouton, Ascenseur ascenseur, Poignet poignet) {
    this.ascenseur = ascenseur;
    this.poignet = poignet;

    this.monterBouton = monterBouton;
    this.descendreBouton = descendreBouton;
    modeManuel = false;

    addRequirements(ascenseur);
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

    //Appuyer sur le dPad = mode manuel
    if(monter || descendre){
      modeManuel = true;
    }

    if (!modeManuel) {// PID automatique vers le sol
      if (poignet.getAngle() < -30 && ascenseur.getPositionExterne() < 0.13) {// Sécurité pour empècher de smasher la
                                                                              // pince dans le bumper
        ascenseur.hold();
      } else if(ascenseur.getPositionExterne() > 0.03){
        ascenseur.setPID(Hauteur.sol[0]);
      }else{
        ascenseur.stop();
      }
    }

    //sinon, dPad jusqu'à ce que la fonction soit caller à nouveau
    else{
      if(monter){
        ascenseur.monter();
      }
      else if(descendre){
        ascenseur.descendre();
      }
      else{
        ascenseur.hold();
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
    ascenseur.stop();
    modeManuel = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;

public class GoToHauteur extends Command {
  private DoubleSupplier cibleAscenceur;
  private DoubleSupplier ciblePoignet;
  private Ascenseur ascenseur;
  private Poignet poignet;


  //Double PID pour le poignet et l'ascenseur
  //Commande qui n'a pas de fin.
  //On hold jusqu'à ce que la commande soit interrompue
  public GoToHauteur(DoubleSupplier cibleAscenceur, DoubleSupplier ciblePoignet, Ascenseur ascenseur, Poignet poignet) {
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
    SmartDashboard.putNumber("cible ascenceur", cibleAscenceur.getAsDouble());
    SmartDashboard.putNumber("cible poignet", ciblePoignet.getAsDouble()); 

    if(ascenseur.atCible()){
      ascenseur.hold();
    }else{
      ascenseur.setPID(cibleAscenceur.getAsDouble());
    }

    if(poignet.atCible()){
      poignet.hold();
    }else{
      poignet.setPID(ciblePoignet.getAsDouble());
    }

  }

  @Override
  public void end(boolean interrupted) {
    ascenseur.stop();
    poignet.stop();
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}

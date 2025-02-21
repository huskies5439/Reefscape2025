// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GamePositions;
import frc.robot.subsystems.BasePilotable;

public class FollowPathVariable extends Command {
  BasePilotable basePilotable;
  DoubleSupplier joystickSupplier;
  double joystick;
  boolean isProcesseur;
  Pose2d cibleActuelle;

  Pose2d cibleGauche;
  Pose2d cibleCentre;
  Pose2d cibleDroite;

  boolean joystickAppuye;
  boolean shiftGauche;
  boolean shiftDroite;

  public FollowPathVariable(boolean isProcesseur, DoubleSupplier joystickSupplier, BasePilotable basePilotable) {
    this.basePilotable = basePilotable;
    this.joystickSupplier = joystickSupplier;
    this.isProcesseur = isProcesseur;

    addRequirements(basePilotable);
  }

  @Override
  public void initialize() {
  
    if(isProcesseur){
      cibleGauche = GamePositions.BlueCoralStationProcLoin;
      cibleCentre = GamePositions.BlueCoralStationProcCentre;
      cibleDroite = GamePositions.BlueCoralStationProcProche;
    }
    else{//Côté cage = sens inversé
      cibleGauche = GamePositions.BlueCoralStationCageProche;
      cibleCentre = GamePositions.BlueCoralStationCageCentre;
      cibleDroite = GamePositions.BlueCoralStationCageLoin;
    }

    //Par défaut, on va au centre
    cibleActuelle = cibleCentre;

    joystickAppuye = false;//Sert à savoir si le joystick est présentement appuyé pour shifter seulement une fois
    shiftGauche = false;
    shiftDroite = false;

  }

  @Override
  public void execute() {

    //Lecture du joystick pour savoir si on change de cible
    joystick = joystickSupplier.getAsDouble();

    if(joystick > 0.9 && !joystickAppuye){//Shift à droite
      joystickAppuye = true;
      shiftDroite = true;
    }
    else if (joystick  < -0.9 && !joystickAppuye){//Shift à gauche
      joystickAppuye = true;
      shiftGauche = true;
    }
    else if (Math.abs(joystick)<=0.9 && joystickAppuye){//Reset au centre
      joystickAppuye = false;
      shiftGauche = false;
      shiftDroite = false;
    }
    
    //Shifter la cible
    if(shiftGauche){
      if(cibleActuelle == cibleDroite){
        cibleActuelle = cibleCentre;
      }
      else if(cibleActuelle == cibleCentre){
        cibleActuelle = cibleGauche;
      }
    }


    else if(shiftDroite){
      if(cibleActuelle == cibleGauche){
        cibleActuelle = cibleCentre;
      }
      else if(cibleActuelle == cibleCentre){
        cibleActuelle = cibleDroite;
      }
    }

    //FollowPath à la cible
    basePilotable.followPath(cibleActuelle);

  }

  @Override
  public void end(boolean interrupted) {
    basePilotable.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

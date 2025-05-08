// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.CorailManip;
import frc.robot.subsystems.Del;

public class GestionDEL extends Command {

  Del del; 
  CorailManip corailManip; 
  //AlgueManip algueManip; 
  BasePilotable basePilotable; 
  
  public GestionDEL(Del del, CorailManip corailManip, /*AlgueManip algueManip,*/BasePilotable basePilotable) {
    this.del = del; 
    this.corailManip = corailManip; 
    //this.algueManip = algueManip; 
    this.basePilotable = basePilotable; 
    addRequirements(del); 
  }

  
  @Override
  public void initialize() {
   
  }


  @Override
  public void execute() {
    if(corailManip.isCorail() /*|| algueManip.isAlgue()*/){//Affiche vert quand on a gobé un élément de jeu
      del.couleur(Color.kRed);//Red et Green sont inversés.... Est-ce dans la librairie ou notre strip ?
    }
    else if(basePilotable.isProcheStationCage() || basePilotable.isProcheStationProcesseur()){
      del.couleur(Color.kPurple);
    }
    else{//Sinon, on fait juste afficher notre couleur d'alliance pour fitter avec les bumpers.
      if(basePilotable.isRedAlliance()){
        del.breathe(Color.kGreen);
      }
      else{
        del.breathe(Color.kBlue);
      }
    }
   
  }


  @Override
  public void end(boolean interrupted) {}

 
  @Override
  public boolean isFinished() {
    return false;
  }
}

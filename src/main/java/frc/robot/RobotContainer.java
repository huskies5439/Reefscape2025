// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.BoutonOperateur;
import frc.robot.Constants.Hauteur;
import frc.robot.Constants.Branche;
import frc.robot.commands.GoToHauteur;
import frc.robot.commands.SetHauteur;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.CorailManip;
import frc.robot.subsystems.Poignet;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final BasePilotable basePilotable = new BasePilotable();
  private final Ascenseur ascenseur = new Ascenseur();
  //private final Poignet poignet = new Poignet();
  //private final AlgueManip algueManip = new AlgueManip(); 
  //private final CorailManip corailManip = new CorailManip(); 

  CommandXboxController manette = new CommandXboxController(0);

  CommandGenericHID operateur = new CommandGenericHID(1);

  public RobotContainer() {
    configureButtonBindings();

    // Commandes par défaut
     basePilotable.setDefaultCommand(
        Commands.run(
            () -> basePilotable.conduire(
                manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                true, true),
            basePilotable));
  }

  private void configureButtonBindings() {

    //manette.a().whileTrue(new GoToHauteur(ascenseur, poignet));

    //  operateur.button(BoutonOperateur.L1).onTrue(new SetHauteur(Hauteur.corailL1, ascenseur, poignet));
    //  operateur.button(BoutonOperateur.L2).onTrue(new SetHauteur(Hauteur.corailL2, ascenseur, poignet));
    //  operateur.button(BoutonOperateur.L3).onTrue(new SetHauteur(Hauteur.corailL3, ascenseur, poignet));
    //  operateur.button(BoutonOperateur.L4).onTrue(new SetHauteur(Hauteur.corailL4, ascenseur, poignet));
    
    
    //Matisse est passé par ici :) 
    operateur.button(BoutonOperateur.A).onTrue(basePilotable.setCibleRecifCommand(Branche.A));
    operateur.button(BoutonOperateur.B).onTrue(basePilotable.setCibleRecifCommand(Branche.B));
    operateur.button(BoutonOperateur.C).onTrue(basePilotable.setCibleRecifCommand(Branche.C));
    operateur.button(BoutonOperateur.D).onTrue(basePilotable.setCibleRecifCommand(Branche.D));
    operateur.button(BoutonOperateur.E).onTrue(basePilotable.setCibleRecifCommand(Branche.E));
    operateur.button(BoutonOperateur.F).onTrue(basePilotable.setCibleRecifCommand(Branche.F));
    operateur.button(BoutonOperateur.G).onTrue(basePilotable.setCibleRecifCommand(Branche.G));
    operateur.button(BoutonOperateur.H).onTrue(basePilotable.setCibleRecifCommand(Branche.H));
    operateur.button(BoutonOperateur.I).onTrue(basePilotable.setCibleRecifCommand(Branche.I));
    operateur.button(BoutonOperateur.J).onTrue(basePilotable.setCibleRecifCommand(Branche.J));
    operateur.button(BoutonOperateur.K).onTrue(basePilotable.setCibleRecifCommand(Branche.K));
    operateur.button(BoutonOperateur.L).onTrue(basePilotable.setCibleRecifCommand(Branche.L)); 
    

    // manette.a().whileTrue(Commands.runEnd(()->ascenseur.setPID(1), ()-> ascenseur.stop(), ascenseur));
    // manette.b().whileTrue(Commands.runEnd(()->ascenseur.setPID(0), ()-> ascenseur.stop(), ascenseur));
    
    // manette.x().whileTrue(Commands.runEnd(()->poignet.setPID(0), ()-> poignet.stop(), poignet));
    // manette.y().whileTrue(Commands.runEnd(()->poignet.setPID(90), ()-> poignet.stop(), poignet));

    /*manette.a().whileTrue(Commands.startEnd(()-> algueManip.gober(), ()-> algueManip.stop(), algueManip));
    manette.b().whileTrue(Commands.startEnd(()-> algueManip.lancer(), ()-> algueManip.stop(), algueManip));

    manette.x().whileTrue(Commands.startEnd(()-> corailManip.gober(), ()-> corailManip.stop(), corailManip));
    manette.y().whileTrue(Commands.startEnd(()-> corailManip.lancer(), ()-> corailManip.stop(), corailManip));*/
    

    manette.povUp().whileTrue(Commands.startEnd(()-> ascenseur.monter(), ()-> ascenseur.stop(), ascenseur));
    manette.povDown().whileTrue(Commands.startEnd(()-> ascenseur.descendre(), ()-> ascenseur.stop(), ascenseur));
    /*manette.povRight().whileTrue(Commands.runEnd(()-> poignet.monter(), ()-> poignet.stop(), poignet));
    manette.povLeft().whileTrue(Commands.runEnd(()-> poignet.descendre(), ()-> poignet.stop(), poignet));*/
  }

  public Command getAutonomousCommand() {
    return null;
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.BoutonOperateur;
import frc.robot.Constants.Hauteur;
import frc.robot.Constants.Branche;
import frc.robot.commands.ActiverGrimpeur;
import frc.robot.commands.AutoCorail;
import frc.robot.commands.GoToHauteur;
import frc.robot.commands.SetHauteur;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.CorailManip;
import frc.robot.subsystems.Poignet;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.EventMarker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final BasePilotable basePilotable = new BasePilotable();
  private final Ascenseur ascenseur = new Ascenseur();
  private final Poignet poignet = new Poignet();

  private final AlgueManip algueManip = new AlgueManip();
  private final CorailManip corailManip = new CorailManip();
  

  CommandXboxController manette = new CommandXboxController(0);

  CommandGenericHID operateur = new CommandGenericHID(1);

  Trigger grimpeurTrigger = manette.leftBumper().and(manette.leftTrigger());
  private boolean pretAGrimper = false;
  Trigger pretAGrimperTrigger = new Trigger(() -> pretAGrimper);

  public RobotContainer() {
    configureButtonBindings();

    // Commandes par défaut
    basePilotable.setDefaultCommand(
        Commands.run(
            () -> basePilotable.conduire(
                manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                true, true),
            basePilotable));

      // commmandes pour pathPlanner 
      
      NamedCommands.registerCommand("SortirCorail", corailManip.sortirCommand());
     // new EventTrigger("GoberCorail").onTrue(corailManip.goberCommand());
      NamedCommands.registerCommand("GoberCorail", corailManip.goberCommand());
    
      NamedCommands.registerCommand("SortirAlgue", algueManip.sortirCommand());
      //new EventTrigger("GoberAlgue").onTrue(algueManip.goberCommand());
      NamedCommands.registerCommand("GoberAlgue", algueManip.goberCommand()); 


  }

  private void configureButtonBindings() {

    manetteOperateur();

    // manette.a().whileTrue(new GoToHauteur(ascenseur, poignet));

    grimpeurTrigger.and(pretAGrimperTrigger.negate())
        .whileTrue(new ActiverGrimpeur(ascenseur, poignet).andThen(() -> pretAGrimper = true));
    grimpeurTrigger.and(pretAGrimperTrigger)
        .whileTrue(Commands.run(() -> ascenseur.descendreAjustable(manette.getRightTriggerAxis()), ascenseur));
    grimpeurTrigger.and(manette.rightBumper())
        .whileTrue(new ActiverGrimpeur(ascenseur, poignet).andThen(() -> pretAGrimper = false));

    manette.a().whileTrue(Commands.runEnd(() -> ascenseur.setPID(0.3), () -> ascenseur.stop(), ascenseur));
    // manette.b().whileTrue(Commands.runEnd(()->ascenseur.setPID(0), ()->
    // ascenseur.stop(), ascenseur));

    // manette.x().whileTrue(Commands.runEnd(()->poignet.setPID(0), ()->
    // poignet.stop(), poignet));
    // manette.y().whileTrue(Commands.runEnd(()->poignet.setPID(90), ()->
    // poignet.stop(), poignet));


    manette.povUp()
        .whileTrue(Commands.startEnd(() -> ascenseur.monter(), () -> ascenseur.setVoltage(Constants.kG), ascenseur));
    manette.povDown().whileTrue(Commands.startEnd(() -> ascenseur.descendre(), () -> ascenseur.stop(), ascenseur));

    // manette.povRight().whileTrue(Commands.runEnd(()-> poignet.monter(), ()->
    // poignet.stop(), poignet));
    // manette.povLeft().whileTrue(Commands.runEnd(()-> poignet.descendre(), ()->
    // poignet.stop(), poignet));

  }

  public Command getAutonomousCommand() {
    return null;
  }

  private void manetteOperateur() {
    // boutton manette oprateur
    operateur.button(BoutonOperateur.L1).onTrue(new SetHauteur(Hauteur.L1, ascenseur, poignet));
    operateur.button(BoutonOperateur.L2).onTrue(new SetHauteur(Hauteur.L2, ascenseur, poignet));
    operateur.button(BoutonOperateur.L3).onTrue(new SetHauteur(Hauteur.L3, ascenseur, poignet));
    operateur.button(BoutonOperateur.L4).onTrue(new SetHauteur(Hauteur.L4, ascenseur, poignet));

    operateur.button(BoutonOperateur.A).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.A));
    operateur.button(BoutonOperateur.B).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.B));
    operateur.button(BoutonOperateur.C).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.C));
    operateur.button(BoutonOperateur.D).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.D));
    operateur.button(BoutonOperateur.E).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.E));
    operateur.button(BoutonOperateur.F).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.F));
    operateur.button(BoutonOperateur.G).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.G));
    operateur.button(BoutonOperateur.H).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.H));
    operateur.button(BoutonOperateur.I).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.I));
    operateur.button(BoutonOperateur.J).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.J));
    operateur.button(BoutonOperateur.K).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.K));
    operateur.button(BoutonOperateur.L).onTrue(basePilotable.setCibleManetteOperateurCommand(Branche.L));

  }

}

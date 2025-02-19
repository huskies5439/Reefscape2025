// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.BoutonOperateur;
import frc.robot.Constants.Hauteur;
import frc.robot.Constants.Branche;
import frc.robot.Constants.GamePositions;
import frc.robot.commands.GoToHauteur;
import frc.robot.commands.PreparationPit;
import frc.robot.commands.SetHauteur;
import frc.robot.commands.grimpeur.ActiverGrimpeur;
import frc.robot.commands.pathplanner.ActionProcesseurPathPlanner;
import frc.robot.commands.pathplanner.ActionRecifAlgueBasPathPlanner;
import frc.robot.commands.pathplanner.ActionRecifAlgueHautPathPlanner;
import frc.robot.commands.pathplanner.ActionRecifCorailPathPlanner;
import frc.robot.commands.pathplanner.ActionStationCagePathPlanner;
import frc.robot.commands.pathplanner.ActionStationProcesseurPathPlanner;
import frc.robot.commands.sequence.AutoCorail;
import frc.robot.commands.sequence.AutoProcesseur;
import frc.robot.commands.sequence.AutoStation;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.CorailManip;
import frc.robot.subsystems.Poignet;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  Trigger stationCageTrigger = new Trigger(basePilotable :: isStationCage); 

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    FollowPathCommand.warmupCommand().schedule(); // warm up la librairie pour éviter les temps d'attente

    // Commandes par défaut
    basePilotable.setDefaultCommand(
        Commands.run(
            () -> basePilotable.conduire(
                manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                true, true),
            basePilotable));
    algueManip.setDefaultCommand(new ConditionalCommand(Commands.runOnce(algueManip::hold, algueManip),
        Commands.runOnce(algueManip::stop, algueManip), algueManip::isAlgue));

    // commmandes pour pathPlanner
    NamedCommands.registerCommand("goberAlgue", algueManip.goberCommand());
    NamedCommands.registerCommand("sortirAlgue", algueManip.sortirCommand().withTimeout(1));

    NamedCommands.registerCommand("goberCorail", corailManip.goberCommand());
    NamedCommands.registerCommand("sortirCorail", corailManip.sortirCommand().withTimeout(1));

    NamedCommands.registerCommand("monterAlgueBas",
        new GoToHauteur(() -> Hauteur.algueBas[0], () -> Hauteur.algueBas[1], ascenseur, poignet));

    NamedCommands.registerCommand("actionProcesseur",
        new ActionProcesseurPathPlanner(basePilotable, ascenseur, poignet));
    NamedCommands.registerCommand("actionRecifAlgueBas",
        new ActionRecifAlgueBasPathPlanner(basePilotable, ascenseur, poignet));
    NamedCommands.registerCommand("actionRecifAlgueHaut",
        new ActionRecifAlgueHautPathPlanner(basePilotable, ascenseur, poignet));
    NamedCommands.registerCommand("actionRecifCorail",
        new ActionRecifCorailPathPlanner(basePilotable, ascenseur, poignet));
    NamedCommands.registerCommand("actionStationCage",
        new ActionStationCagePathPlanner(basePilotable, ascenseur, poignet));
    NamedCommands.registerCommand("actionStationProcesseur",
        new ActionStationProcesseurPathPlanner(basePilotable, ascenseur, poignet));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureButtonBindings() {

    manetteOperateur();

    // manette.a().whileTrue(new GoToHauteur(() -> ascenseur.getCibleManetteOperateur(),
    //     () -> poignet.getCibleManetteOperateur(), ascenseur, poignet));

    // manette.a().onTrue(Commands.runEnd(() ->
    // ascenseur.setPID(ascenseur.getCibleManetteOperateur()), () ->
    // ascenseur.hold(), ascenseur));
    // manette.b().whileTrue(Commands.runEnd(()->ascenseur.setPID(0), ()->
    // ascenseur.stop(), ascenseur));

    // manette.a().whileTrue(Commands.runEnd(() ->
    // poignet.setPID(poignet.getCibleManetteOperateur()), () -> poignet.hold(),
    // poignet));
    // manette.b().whileTrue(Commands.runEnd(()->poignet.setPID(45), ()->
    // poignet.hold(), poignet));

    // manette.x().whileTrue(Commands.runEnd(()->poignet.setPID(0), ()->
    // poignet.hold(), poignet));
    // // manette.y().whileTrue(Commands.runEnd(()->poignet.setPID(90), ()->
    // // poignet.stop(), poignet));

    // manette.y().whileTrue(Commands.runEnd(()->poignet.setPID(-45
    // ), ()->poignet.hold(), poignet));

    manette.a().whileTrue(new AutoCorail(basePilotable, ascenseur, poignet));
    manette.b().and(stationCageTrigger).whileTrue(new AutoStation(GamePositions.BlueCoralStationCage,basePilotable, ascenseur, poignet));
    manette.b().and(stationCageTrigger.negate()).whileTrue(new AutoStation(GamePositions.BlueCoralStationProc,basePilotable, ascenseur, poignet));
    manette.x().whileTrue(new AutoProcesseur(basePilotable, ascenseur, poignet));

    manette.povUp()
        .whileTrue(Commands.startEnd(() -> ascenseur.monter(), () -> ascenseur.hold(), ascenseur));
    manette.povDown().whileTrue(Commands.startEnd(() -> ascenseur.descendre(), () -> ascenseur.hold(), ascenseur));

    manette.povRight().whileTrue(Commands.startEnd(() -> poignet.monter(), () -> poignet.stop(), poignet));
    manette.povLeft().whileTrue(Commands.startEnd(() -> poignet.descendre(), () -> poignet.stop(), poignet));

    // manette.x().whileTrue(corailManip.goberCommand());
    // manette.y().whileTrue(corailManip.sortirCommand());

    // manette.x().whileTrue(algueManip.goberCommand());
    // manette.y().whileTrue(algueManip.sortirCommand());

    manette.start().onTrue(new PreparationPit(ascenseur, poignet));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
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

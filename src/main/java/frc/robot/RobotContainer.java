// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Algue;
import frc.robot.Constants.BoutonOperateur;
import frc.robot.Constants.Hauteur;
import frc.robot.Constants.Branche;
import frc.robot.Constants.GamePositions;
import frc.robot.commands.GoToHauteur;
import frc.robot.commands.PreparationPit;
import frc.robot.commands.SetHauteur;
import frc.robot.commands.grimpeur.ActiverGrimpeur;
import frc.robot.commands.grimpeur.ControleGrimpeur;
import frc.robot.commands.pathplanner.ActionProcesseurPathPlanner;
import frc.robot.commands.pathplanner.ActionRecifAlgueBasPathPlanner;
import frc.robot.commands.pathplanner.ActionRecifAlgueHautPathPlanner;
import frc.robot.commands.pathplanner.ActionRecifCorailPathPlanner;
import frc.robot.commands.pathplanner.ActionStationCagePathPlanner;
import frc.robot.commands.pathplanner.ActionStationProcesseurPathPlanner;
import frc.robot.commands.sequence.AutoAlgue;
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

  Trigger grimpeurTrigger = manette.leftBumper().and(manette.rightBumper());
  private boolean pretAGrimper = false;
  Trigger pretAGrimperTrigger = new Trigger(() -> pretAGrimper);

  Trigger stationCageTrigger = new Trigger(basePilotable :: isStationCage); 

  Trigger manetteA = manette.a(); 
  Trigger manetteY = manette.y(); 

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
    boutonCorail();
    boutonAlgue();

    manette.b().and(stationCageTrigger).whileTrue(new AutoStation(GamePositions.BlueCoralStationCage,basePilotable, ascenseur, poignet));
    manette.b().and(stationCageTrigger.negate()).whileTrue(new AutoStation(GamePositions.BlueCoralStationProc,basePilotable, ascenseur, poignet));
    manette.x().whileTrue(new AutoProcesseur(basePilotable, ascenseur, poignet));

    manette.leftBumper().whileTrue(corailManip.sortirCommand());
    manette.rightBumper().whileTrue(algueManip.sortirCommand());

    grimpeurTrigger.toggleOnTrue(new ActiverGrimpeur(ascenseur,poignet).andThen(new ControleGrimpeur(manette::getLeftTriggerAxis, manette::getRightTriggerAxis,ascenseur )));

    manette.povUp()
        .whileTrue(Commands.startEnd(() -> ascenseur.monter(), () -> ascenseur.hold(), ascenseur));
    manette.povDown().whileTrue(Commands.startEnd(() -> ascenseur.descendre(), () -> ascenseur.hold(), ascenseur));

    manette.povRight().whileTrue(Commands.startEnd(() -> poignet.monter(), () -> poignet.stop(), poignet));
    manette.povLeft().whileTrue(Commands.startEnd(() -> poignet.descendre(), () -> poignet.stop(), poignet));

    // manette.x().whileTrue(corailManip.goberCommand());
    // manette.x().whileTrue(algueManip.goberCommand());

    manette.start().whileTrue(Commands.runEnd(()->ascenseur.setPID(0.25), ()->ascenseur.hold(), ascenseur));
    

    // manette.start().onTrue(new PreparationPit(ascenseur, poignet));
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


  }

private void boutonCorail() {
    operateur.button(BoutonOperateur.A).and(manetteA).whileTrue(new AutoCorail(Branche.A, basePilotable, ascenseur, poignet));
    operateur.button(BoutonOperateur.B).and(manetteA).whileTrue(new AutoCorail(Branche.B, basePilotable, ascenseur, poignet));
    operateur.button(BoutonOperateur.C).and(manetteA).whileTrue(new AutoCorail(Branche.C, basePilotable, ascenseur, poignet));
    operateur.button(BoutonOperateur.D).and(manetteA).whileTrue(new AutoCorail(Branche.D, basePilotable, ascenseur, poignet));
    operateur.button(BoutonOperateur.E).and(manetteA).whileTrue(new AutoCorail(Branche.E, basePilotable, ascenseur, poignet));
    operateur.button(BoutonOperateur.F).and(manetteA).whileTrue(new AutoCorail(Branche.F, basePilotable, ascenseur, poignet));
    operateur.button(BoutonOperateur.G).and(manetteA).whileTrue(new AutoCorail(Branche.G, basePilotable, ascenseur, poignet));
    operateur.button(BoutonOperateur.H).and(manetteA).whileTrue(new AutoCorail(Branche.H, basePilotable, ascenseur, poignet));
    operateur.button(BoutonOperateur.I).and(manetteA).whileTrue(new AutoCorail(Branche.I, basePilotable, ascenseur, poignet));
    operateur.button(BoutonOperateur.J).and(manetteA).whileTrue(new AutoCorail(Branche.J, basePilotable, ascenseur, poignet));
    operateur.button(BoutonOperateur.K).and(manetteA).whileTrue(new AutoCorail(Branche.K, basePilotable, ascenseur, poignet));
    operateur.button(BoutonOperateur.L).and(manetteA).whileTrue(new AutoCorail(Branche.L, basePilotable, ascenseur, poignet));
}

private void boutonAlgue(){
    manetteY.and(operateur.button(BoutonOperateur.A).or(operateur.button(BoutonOperateur.B)))
                .whileTrue(new AutoAlgue(Algue.AB, Hauteur.algueHaut, ascenseur, poignet, basePilotable, algueManip));
    manetteY.and(operateur.button(BoutonOperateur.C).or(operateur.button(BoutonOperateur.D)))
                .whileTrue(new AutoAlgue(Algue.CD, Hauteur.algueBas, ascenseur, poignet, basePilotable, algueManip));
    manetteY.and(operateur.button(BoutonOperateur.E).or(operateur.button(BoutonOperateur.F)))
                .whileTrue(new AutoAlgue(Algue.EF, Hauteur.algueHaut, ascenseur, poignet, basePilotable, algueManip));
    manetteY.and(operateur.button(BoutonOperateur.G).or(operateur.button(BoutonOperateur.H)))
                .whileTrue(new AutoAlgue(Algue.GH, Hauteur.algueBas, ascenseur, poignet, basePilotable, algueManip));
    manetteY.and(operateur.button(BoutonOperateur.I).or(operateur.button(BoutonOperateur.J)))
                .whileTrue(new AutoAlgue(Algue.IJ, Hauteur.algueHaut, ascenseur, poignet, basePilotable, algueManip));
    manetteY.and(operateur.button(BoutonOperateur.K).or(operateur.button(BoutonOperateur.L)))
                .whileTrue(new AutoAlgue(Algue.KL, Hauteur.algueBas, ascenseur, poignet, basePilotable, algueManip));

    }

}

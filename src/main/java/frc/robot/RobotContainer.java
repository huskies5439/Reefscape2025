// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Algue;
import frc.robot.Constants.BoutonOperateur;
import frc.robot.Constants.Hauteur;
import frc.robot.Constants.Branche;
import frc.robot.Constants.GamePositions;
import frc.robot.commands.AscenseurDefaut;
import frc.robot.commands.GoToHauteur;
import frc.robot.commands.PoignetDefaut;
import frc.robot.commands.PreparationPit;
import frc.robot.commands.SetHauteur;
import frc.robot.commands.grimpeur.ActiverGrimpeur;
import frc.robot.commands.grimpeur.ControleGrimpeur;
import frc.robot.commands.pathplanner.ActionProcesseurPathPlanner;
import frc.robot.commands.pathplanner.ActionRecifAlgueBasPathPlanner;
import frc.robot.commands.pathplanner.ActionRecifAlgueHautPathPlanner;
import frc.robot.commands.pathplanner.ActionRecifCorailPathPlanner;
import frc.robot.commands.pathplanner.ActionRecifCorailPathPlannerL2;
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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

    Trigger grimpeurTrigger = manette.leftTrigger(0.9).and(manette.rightTrigger(0.9));
    boolean modeGrimpeur = false; 
    Trigger modeGrimpeurTrigger = new Trigger(()->modeGrimpeur);

    Trigger stationCageTrigger = new Trigger(basePilotable::isStationCage);
    Trigger stationGaucheTrigger = new Trigger(manette.leftTrigger());
    Trigger stationCentreTrigger = new Trigger(manette.leftTrigger().negate().and(manette.rightTrigger().negate()));
    Trigger stationDroiteTrigger = new Trigger(manette.rightTrigger());

    Trigger procheStationTrigger = new Trigger(()-> basePilotable.isProcheStationCage() || basePilotable.isProcheStationProcesseur()).and(()->{return DriverStation.isTeleop();});
    Trigger isCorailTrigger = new Trigger(corailManip :: isCorail); 

    Trigger manetteA = manette.a();
    Trigger manetteY = manette.y();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

       
        FollowPathCommand.warmupCommand().schedule(); // warm up la librairie pour éviter les temps d'attente

       

        // commmandes pour pathPlanner
        NamedCommands.registerCommand("goberAlgue", algueManip.goberCommand());
        NamedCommands.registerCommand("sortirAlgue", algueManip.sortirCommand().withTimeout(1));

        NamedCommands.registerCommand("goberCorail", new ActionStationProcesseurPathPlanner(basePilotable, ascenseur, poignet, corailManip));
        NamedCommands.registerCommand("sortirCorail", corailManip.sortirCommand()
         .alongWith(Commands.run(ascenseur::hold, ascenseur)).alongWith(Commands.run(poignet::hold, poignet))
         .withTimeout(0.25));

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
         NamedCommands.registerCommand("actionRecifCorailL2",
                new ActionRecifCorailPathPlannerL2(basePilotable, ascenseur, poignet));
        NamedCommands.registerCommand("actionStationCage",
                new ActionStationCagePathPlanner(basePilotable, ascenseur, poignet, corailManip));

       
   
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);


         // Commandes par défaut
         basePilotable.setDefaultCommand(
                Commands.run(
                        () -> basePilotable.conduire(
                                manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                true, true),
                        basePilotable));


        algueManip.setDefaultCommand(new ConditionalCommand(
                Commands.runOnce(algueManip::hold, algueManip),
                Commands.runOnce(algueManip::stop, algueManip), 
                algueManip::isAlgue)
                );

         corailManip.setDefaultCommand(new ConditionalCommand(
                Commands.runOnce(corailManip::hold, corailManip),
                Commands.runOnce(corailManip::stop, corailManip), 
                corailManip::isCorail)
                 );

        //Note SAM : si ça ne marche pas avec le dPad, changer les commandes pour prendre un boolean au lieu d'un BooleanSupplier
        //Les boutons de la manettes sont des Triggers, donc des BooleanSupplier !
        ascenseur.setDefaultCommand(new AscenseurDefaut(manette.povUp(),manette.povDown(), ascenseur, poignet));
        poignet.setDefaultCommand(new PoignetDefaut(manette.povLeft(), manette.povRight(),poignet, algueManip));

        configureButtonBindings();

       
    
    }

    private void configureButtonBindings() {

        manetteOperateur();
        boutonCorail();
        boutonAlgue();

        //Station qui marche, aligné sur la coral station qu'on a au Katimavik
        // manette.b().and(stationCageTrigger)
        //         .whileTrue(new AutoStation(GamePositions.BlueCoralStationCageCentre, basePilotable, ascenseur, poignet,corailManip));
        // manette.b().and(stationCageTrigger.negate())
        //         .whileTrue(new AutoStation(GamePositions.BlueCoralStationProcCentre, basePilotable, ascenseur, poignet,corailManip));

        manette.b().and(stationCageTrigger).and(stationGaucheTrigger).whileTrue(new AutoStation(GamePositions.BlueCoralStationCageProche, basePilotable,ascenseur,poignet,corailManip));
        manette.b().and(stationCageTrigger).and(stationCentreTrigger).whileTrue(new AutoStation(GamePositions.BlueCoralStationCageCentre, basePilotable,ascenseur,poignet,corailManip));
        manette.b().and(stationCageTrigger).and(stationDroiteTrigger).whileTrue(new AutoStation(GamePositions.BlueCoralStationCageLoin, basePilotable,ascenseur,poignet,corailManip));

        manette.b().and(stationCageTrigger.negate()).and(stationGaucheTrigger).whileTrue(new AutoStation(GamePositions.BlueCoralStationProcLoin, basePilotable,ascenseur,poignet,corailManip));
        manette.b().and(stationCageTrigger.negate()).and(stationCentreTrigger).whileTrue(new AutoStation(GamePositions.BlueCoralStationProcCentre, basePilotable,ascenseur,poignet,corailManip));
        manette.b().and(stationCageTrigger.negate()).and(stationDroiteTrigger).whileTrue(new AutoStation(GamePositions.BlueCoralStationProcProche, basePilotable,ascenseur,poignet,corailManip));

        manette.x().and(modeGrimpeurTrigger.negate()).whileTrue(new AutoProcesseur(basePilotable, ascenseur, poignet));
        
        manette.rightBumper().whileTrue(algueManip.sortirCommand().alongWith(corailManip.sortirCommand()));
        manette.leftBumper().whileTrue(algueManip.goberCommand().alongWith(new GoToHauteur(()-> Hauteur.algueSol[0], ()-> Hauteur.algueSol[1], ascenseur, poignet)));

        grimpeurTrigger.onTrue(Commands.runOnce(()->{modeGrimpeur = !modeGrimpeur;})); 

        modeGrimpeurTrigger.whileTrue(new ActiverGrimpeur(ascenseur, poignet)
                .andThen(new ControleGrimpeur(manette::getLeftTriggerAxis, manette::getRightTriggerAxis, manette.x(), ascenseur)));

        manette.x().and(modeGrimpeurTrigger).toggleOnTrue(Commands.startEnd(ascenseur::barrer, ascenseur::debarrer)); 

       procheStationTrigger.whileTrue(new GoToHauteur(()-> Hauteur.station[0], ()-> Hauteur.station[1], ascenseur, poignet).alongWith(corailManip.goberCommand()).until(isCorailTrigger)); 

        //dPad pour ajuster le bouton. Normalement ça devrait être intégré aux commandes par défaut pour plus de sécurité.
        // manette.povUp().whileTrue(Commands.startEnd(() -> ascenseur.monter(), () -> ascenseur.hold(), ascenseur));
        // manette.povDown().whileTrue(Commands.startEnd(() -> ascenseur.descendre(), () -> ascenseur.hold(), ascenseur));

        // manette.povRight().whileTrue(Commands.startEnd(() -> poignet.monter(), () -> poignet.stop(), poignet));
        // manette.povLeft().whileTrue(Commands.startEnd(() -> poignet.descendre(), () -> poignet.stop(), poignet));


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

    }

    private void boutonCorail() {
        manetteA.and(operateur.button(BoutonOperateur.A))
                .whileTrue(new AutoCorail(Branche.A, basePilotable, ascenseur, poignet));

        manetteA.and(operateur.button(BoutonOperateur.B))
                .whileTrue(new AutoCorail(Branche.B, basePilotable, ascenseur, poignet));

        manetteA.and(operateur.button(BoutonOperateur.C))
                .whileTrue(new AutoCorail(Branche.C, basePilotable, ascenseur, poignet));

        manetteA.and(operateur.button(BoutonOperateur.D))
                .whileTrue(new AutoCorail(Branche.D, basePilotable, ascenseur, poignet));

        manetteA.and(operateur.button(BoutonOperateur.E))
                .whileTrue(new AutoCorail(Branche.E, basePilotable, ascenseur, poignet));

        manetteA.and(operateur.button(BoutonOperateur.F))
                .whileTrue(new AutoCorail(Branche.F, basePilotable, ascenseur, poignet));

        manetteA.and(operateur.button(BoutonOperateur.G))
                .whileTrue(new AutoCorail(Branche.G, basePilotable, ascenseur, poignet));

        manetteA.and(operateur.button(BoutonOperateur.H))
                .whileTrue(new AutoCorail(Branche.H, basePilotable, ascenseur, poignet));

        manetteA.and(operateur.button(BoutonOperateur.I))
                .whileTrue(new AutoCorail(Branche.I, basePilotable, ascenseur, poignet));

        manetteA.and(operateur.button(BoutonOperateur.J))
                .whileTrue(new AutoCorail(Branche.J, basePilotable, ascenseur, poignet));

        manetteA.and(operateur.button(BoutonOperateur.K))
                .whileTrue(new AutoCorail(Branche.K, basePilotable, ascenseur, poignet));

        manetteA.and(operateur.button(BoutonOperateur.L))
                .whileTrue(new AutoCorail(Branche.L, basePilotable, ascenseur, poignet));

    }

    private void boutonAlgue() {
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

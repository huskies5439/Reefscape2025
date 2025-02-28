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
import frc.robot.commands.GestionDEL;
import frc.robot.commands.GoToHauteur;
import frc.robot.commands.PoignetDefaut;
import frc.robot.commands.PreparationPit;
import frc.robot.commands.SetHauteur;
import frc.robot.commands.grimpeur.ActiverGrimpeur;
import frc.robot.commands.grimpeur.ControleGrimpeur;
import frc.robot.commands.pathplanner.ActionRecifCorailPathPlanner;
import frc.robot.commands.pathplanner.ActionRecifCorailPathPlannerL2;
import frc.robot.commands.pathplanner.ActionStationPathPlanner;
import frc.robot.commands.sequence.AutoAlgue;
import frc.robot.commands.sequence.AutoCorail;
import frc.robot.commands.sequence.AutoProcesseur;
import frc.robot.commands.sequence.AutoStation;
import frc.robot.subsystems.AlgueManip;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.CorailManip;
import frc.robot.subsystems.Del;
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

        // créer les sous systèmes
        private final BasePilotable basePilotable = new BasePilotable();
        private final Ascenseur ascenseur = new Ascenseur();
        private final Poignet poignet = new Poignet();
        private final AlgueManip algueManip = new AlgueManip();
        private final CorailManip corailManip = new CorailManip();
        private final Del del = new Del();

        CommandXboxController manette = new CommandXboxController(0);

        CommandGenericHID operateur = new CommandGenericHID(1);

        Trigger grimpeurTrigger = manette.leftTrigger(0.9).and(manette.rightTrigger(0.9));
        boolean modeGrimpeur = false;
        Trigger modeGrimpeurTrigger = new Trigger(() -> modeGrimpeur);

        Trigger stationCageTrigger = new Trigger(basePilotable::isStationCage);
        Trigger stationGaucheTrigger = new Trigger(manette.leftTrigger());
        Trigger stationCentreTrigger = new Trigger(manette.leftTrigger().negate().and(manette.rightTrigger().negate()));
        Trigger stationDroiteTrigger = new Trigger(manette.rightTrigger());

        Trigger procheStationTrigger = new Trigger(
                        () -> basePilotable.isProcheStationCage() || basePilotable.isProcheStationProcesseur())
                        .and(() -> {
                                return DriverStation.isTeleop();
                        });
        Trigger isCorailTrigger = new Trigger(corailManip::isCorail);

        Trigger manetteA = manette.a();
        Trigger manetteX = manette.x();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                FollowPathCommand.warmupCommand().schedule(); // warm up la librairie pour éviter les temps d'attente

                // commmandes pour pathPlanner
                // LES NAMED COMMANDS DOIVENT ÊTRE CALLER AVANT LES AUTOS, DONC AVANT LE
                // SENDABLE CHOOSER
                NamedCommands.registerCommand("goberCorail",
                                new ActionStationPathPlanner(basePilotable, ascenseur, poignet, corailManip));
                NamedCommands.registerCommand("sortirCorail", corailManip.sortirCommand()
                                .alongWith(Commands.run(ascenseur::hold, ascenseur))
                                .alongWith(Commands.run(poignet::hold, poignet))
                                .withTimeout(0.25));

                NamedCommands.registerCommand("actionRecifCorail",
                                new ActionRecifCorailPathPlanner(basePilotable, ascenseur, poignet));
                NamedCommands.registerCommand("actionRecifCorailL2",
                                new ActionRecifCorailPathPlannerL2(basePilotable, ascenseur, poignet));

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                // Commandes par défaut (lorsque le robot s'ennuie)
                basePilotable.setDefaultCommand(
                                Commands.run(
                                                () -> basePilotable.conduire(
                                                                manette.getLeftY(), manette.getLeftX(),
                                                                manette.getRightX(),
                                                                true, true),
                                                basePilotable));

                // des holds pour les sous systèmes
                algueManip.setDefaultCommand(new ConditionalCommand(
                                Commands.runOnce(algueManip::hold, algueManip),
                                Commands.runOnce(algueManip::stop, algueManip),
                                algueManip::isAlgue));

                corailManip.setDefaultCommand(new ConditionalCommand(
                                Commands.runOnce(corailManip::hold, corailManip),
                                Commands.runOnce(corailManip::stop, corailManip),
                                corailManip::isCorail));

                // Les boutons de la manettes sont des Triggers, donc des BooleanSupplier !
                ascenseur.setDefaultCommand(
                                new AscenseurDefaut(manette.povUp(), manette.povDown(), ascenseur, poignet));
                poignet.setDefaultCommand(
                                new PoignetDefaut(manette.povLeft(), manette.povRight(), poignet, algueManip));

                del.setDefaultCommand(new GestionDEL(del, corailManip, algueManip, basePilotable));

                configureButtonBindings();

        }

        private void configureButtonBindings() {

                // voir en bas pour les fonctions
                manetteOperateur();
                boutonCorail();
                boutonAlgue();

                // boutons pour la mannette ;
                // B = auto station 

                manette.b().and(stationCageTrigger).and(stationGaucheTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationCageProche, basePilotable,
                                                ascenseur, poignet, corailManip));
                manette.b().and(stationCageTrigger).and(stationCentreTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationCageCentre, basePilotable,
                                                ascenseur, poignet, corailManip));
                manette.b().and(stationCageTrigger).and(stationDroiteTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationCageLoin, basePilotable,
                                                ascenseur, poignet, corailManip));

                manette.b().and(stationCageTrigger.negate()).and(stationGaucheTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationProcLoin, basePilotable,
                                                ascenseur, poignet, corailManip));
                manette.b().and(stationCageTrigger.negate()).and(stationCentreTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationProcCentre, basePilotable,
                                                ascenseur, poignet, corailManip));
                manette.b().and(stationCageTrigger.negate()).and(stationDroiteTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationProcProche, basePilotable,
                                                ascenseur, poignet, corailManip));

                // Y = Auto algue au processeur 
                manette.y().and(modeGrimpeurTrigger.negate())
                                .whileTrue(new AutoProcesseur(basePilotable, ascenseur, poignet));
               
                 //Sortir les pieces de jeu
                manette.rightBumper().whileTrue(algueManip.sortirCommand().alongWith(corailManip.sortirCommand()));
                
                //Gober les algues 
                manette.leftBumper().whileTrue(algueManip.goberCommand().alongWith(new GoToHauteur(
                                () -> Hauteur.algueSol[0], () -> Hauteur.algueSol[1], ascenseur, poignet)));

                //Activer/desactiver grimpeur 
                grimpeurTrigger.onTrue(Commands.runOnce(() -> {
                        modeGrimpeur = !modeGrimpeur;
                }));

                //Controler grimpeur
                modeGrimpeurTrigger.whileTrue(new ActiverGrimpeur(ascenseur, poignet)
                                .andThen(new ControleGrimpeur(manette::getLeftTriggerAxis, manette::getRightTriggerAxis,
                                                manette.x(), ascenseur))
                                .alongWith(Commands.run(() -> del.rainbow(), del)));
               
                 // Barrer le servo en mode grimpeur
                manette.x().and(modeGrimpeurTrigger)
                                .toggleOnTrue(Commands.startEnd(ascenseur::barrer, ascenseur::debarrer));

                // Trigger pour se mettre en mode Station lorsque proche
                procheStationTrigger.whileTrue(
                                new GoToHauteur(() -> Hauteur.station[0], () -> Hauteur.station[1], ascenseur, poignet)
                                                .alongWith(corailManip.goberCommand()).until(isCorailTrigger));
               
                 //Homing dans le pit 
                manette.start().onTrue(new PreparationPit(ascenseur, poignet));
        }

        //Choisir les paths dans Dashboard 
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

        //Automatiquement placer base pilotable et monter l'échelle pour coraux 
        private void boutonCorail() {
                manetteA.and(operateur.button(BoutonOperateur.A))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.A, basePilotable, ascenseur, poignet));

                manetteA.and(operateur.button(BoutonOperateur.B))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.B, basePilotable, ascenseur, poignet));

                manetteA.and(operateur.button(BoutonOperateur.C))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.C, basePilotable, ascenseur, poignet));

                manetteA.and(operateur.button(BoutonOperateur.D))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.D, basePilotable, ascenseur, poignet));

                manetteA.and(operateur.button(BoutonOperateur.E))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.E, basePilotable, ascenseur, poignet));

                manetteA.and(operateur.button(BoutonOperateur.F))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.F, basePilotable, ascenseur, poignet));

                manetteA.and(operateur.button(BoutonOperateur.G))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.G, basePilotable, ascenseur, poignet));

                manetteA.and(operateur.button(BoutonOperateur.H))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.H, basePilotable, ascenseur, poignet));

                manetteA.and(operateur.button(BoutonOperateur.I))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.I, basePilotable, ascenseur, poignet));

                manetteA.and(operateur.button(BoutonOperateur.J))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.J, basePilotable, ascenseur, poignet));

                manetteA.and(operateur.button(BoutonOperateur.K))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.K, basePilotable, ascenseur, poignet));

                manetteA.and(operateur.button(BoutonOperateur.L))
                                .whileTrue(new AutoCorail(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Branche.L, basePilotable, ascenseur, poignet));

        }

        //Automatiquement chercher des algues selon des hauteurs alternantes 
        private void boutonAlgue() {
                manetteX.and(operateur.button(BoutonOperateur.A).or(operateur.button(BoutonOperateur.B)))
                                .whileTrue(new AutoAlgue(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Algue.AB, Hauteur.algueHaut, ascenseur, poignet, basePilotable,
                                                algueManip));

                manetteX.and(operateur.button(BoutonOperateur.C).or(operateur.button(BoutonOperateur.D)))
                                .whileTrue(new AutoAlgue(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Algue.CD, Hauteur.algueBas, ascenseur, poignet, basePilotable,
                                                algueManip));

                manetteX.and(operateur.button(BoutonOperateur.E).or(operateur.button(BoutonOperateur.F)))
                                .whileTrue(new AutoAlgue(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Algue.EF, Hauteur.algueHaut, ascenseur, poignet, basePilotable,
                                                algueManip));

                manetteX.and(operateur.button(BoutonOperateur.G).or(operateur.button(BoutonOperateur.H)))
                                .whileTrue(new AutoAlgue(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Algue.GH, Hauteur.algueBas, ascenseur, poignet, basePilotable,
                                                algueManip));

                manetteX.and(operateur.button(BoutonOperateur.I).or(operateur.button(BoutonOperateur.J)))
                                .whileTrue(new AutoAlgue(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Algue.IJ, Hauteur.algueHaut, ascenseur, poignet, basePilotable,
                                                algueManip));

                manetteX.and(operateur.button(BoutonOperateur.K).or(operateur.button(BoutonOperateur.L)))
                                .whileTrue(new AutoAlgue(manette.getLeftY(), manette.getLeftX(), manette.getRightX(),
                                                Algue.KL, Hauteur.algueBas, ascenseur, poignet, basePilotable,
                                                algueManip));
        }

}

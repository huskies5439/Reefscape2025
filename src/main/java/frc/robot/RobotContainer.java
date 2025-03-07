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
import frc.robot.commands.BasePilotableDefaut;
import frc.robot.commands.DescenteAutomatique;
import frc.robot.commands.GestionDEL;
import frc.robot.commands.GoToHauteur;
import frc.robot.commands.PoignetDefaut;
import frc.robot.commands.PreparationPit;
import frc.robot.commands.SetHauteur;
import frc.robot.commands.grimpeur.ActiverGrimpeur;
import frc.robot.commands.grimpeur.ControleGrimpeur;
import frc.robot.commands.pathplanner.actionCorailPP;
import frc.robot.commands.pathplanner.actionSationPP;
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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

        // Créer les sous systèmes
        private final BasePilotable basePilotable = new BasePilotable();
        private final Ascenseur ascenseur = new Ascenseur();
        private final Poignet poignet = new Poignet();
        private final AlgueManip algueManip = new AlgueManip();
        private final CorailManip corailManip = new CorailManip();
        private final Del del = new Del();


        //Les deux manettes
        CommandXboxController manette = new CommandXboxController(0);
        CommandGenericHID operateur = new CommandGenericHID(1); //Voir le dossier manetteOperateur pour le code platformio

        //Trigger arbitraire du monde grimpeur
        Trigger grimpeurTrigger = //Les deux triggers de la manette toggle le "mode grimpeur"
                manette.leftTrigger(0.9).and(manette.rightTrigger(0.9));
        
        //Permet le suivi du mode grimpeur. Trop simple cette année pour mettre dans un sous-système
        boolean modeGrimpeur = false;
        Trigger modeGrimpeurTrigger = new Trigger(() -> modeGrimpeur);
        Trigger resetEncodeurTrigger = new Trigger(ascenseur::isLimitSwitch);


        //Trigger arbitraire pour aller à la bonne station selon la moitié de terrain
        Trigger stationCageTrigger = new Trigger(basePilotable::isStationCage);
        
        //3 triggers abitraires pour décaler le robot de gauche à droite sur la station
        Trigger stationGaucheTrigger = new Trigger(manette.leftTrigger());
        Trigger stationCentreTrigger = new Trigger(manette.leftTrigger().negate().and(manette.rightTrigger().negate()));
        Trigger stationDroiteTrigger = new Trigger(manette.rightTrigger());

        
        //Trigger pour se mettre automatiqument en mode gober si on est proche d'une station
        Trigger procheStationTrigger = new Trigger(
                        () -> basePilotable.isProcheStationCage() || basePilotable.isProcheStationProcesseur())
                        .and(() -> {return DriverStation.isTeleop();});//Les triggers arbitraires brisent pathplanner, donc il faut les désactiver en auto


        /* Comme il y a des tonnes de commandes associées à ces deux boutons,
        nous avons fait un trigger arbitraire afin qu'il soit facile de modifer
        le mapping de la manette si nécessaire*/
        Trigger manetteA = manette.a();//Pour autoCorail
        Trigger manetteX = manette.x();//Pour autoAlgue

        
        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                FollowPathCommand.warmupCommand().schedule(); // warm up la librairie pour éviter les temps d'attente

                ////////////////// Commmandes pour PathPlanner/////////////////
                // IMPORTANT : LES NAMED COMMANDS DOIVENT ÊTRE CALLER AVANT LES AUTOS, DONC AVANT LE SENDABLE CHOOSER
                
                //Pour gober. La fonction automatique ne marche pas dans PathPlanner car il ne gère pas les triggers
                NamedCommands.registerCommand("goberCorail",
                                new actionSationPP(basePilotable, ascenseur, poignet, corailManip));
                
                //Pour sortir le corail en gardant l'ascenceur en bonne position.
                //Nécessaire car sinon les commandes par défaut s'effectuent en auto
                NamedCommands.registerCommand("sortirCorail", corailManip.sortirCommand()
                                .alongWith(Commands.run(ascenseur::hold, ascenseur))
                                .alongWith(Commands.run(poignet::hold, poignet))
                                .withTimeout(0.25));

                //Actions à accomplir en se rendant vers le Récif
                NamedCommands.registerCommand("actionRecifCorail",
                                new actionCorailPP(Hauteur.L4, basePilotable, ascenseur, poignet, corailManip));
                NamedCommands.registerCommand("actionRecifCorailL2",
                                new actionCorailPP(Hauteur.L2, basePilotable, ascenseur, poignet, corailManip));

                //Construction du sendable chooser
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);


                /////////////// Commandes par défaut (lorsque le robot s'ennuie)//////////////
                
                //Conduire avec joystick
                basePilotable.setDefaultCommand(new BasePilotableDefaut(manette::getLeftY,
                                                 manette::getLeftX, manette::getRightX, basePilotable, ascenseur));
                              

                //Hold automatique s'il y a une pièce de jeu dans les manipulateurs, sinon stop
                algueManip.setDefaultCommand(new ConditionalCommand(
                                Commands.runOnce(algueManip::hold, algueManip),
                                Commands.runOnce(algueManip::stop, algueManip),
                                algueManip::isAlgue));

                corailManip.setDefaultCommand(new ConditionalCommand(
                                Commands.runOnce(corailManip::hold, corailManip),
                                Commands.runOnce(corailManip::stop, corailManip),
                                corailManip::isCorail));

                
                //Retourne automatiquement à sa position de départ (avec quelques protections)
                //Override le PID si on utilise le DPad pour se déplacer manuellement
                ascenseur.setDefaultCommand(
                                new AscenseurDefaut(manette.povUp(), manette.povDown(), ascenseur, poignet));
                                //Pas besoin de lambda car Button = Trigger = BooleanSupplier !
                poignet.setDefaultCommand(
                                new PoignetDefaut(manette.povLeft(), manette.povRight(), poignet, algueManip));

                del.setDefaultCommand(new GestionDEL(del, corailManip, algueManip, basePilotable));

                configureButtonBindings();

        }

        private void configureButtonBindings() {
                /* Si on met "bêtement" les cibles en argument des fonctions, elles ne se mettent
                 * pas à jour durant la match. En effet, la cible devient constante selon l'état de la manette
                 * opérateur lors de la compilation du code.
                 * 
                 * Il faut donc utiliser un lambda et un supplier. 
                 * C'est ce que l'on fait pour GoToHauteur, car la cible est un double[].
                 * 
                 * Par contre, pour toutes les fonctions qui font du pathfinding jusqu'à une Pose2d,
                 * Il n'y a pas de Pose2dSupplier !!
                 * 
                 * Alors, la stratégie adoptée est de dupliquer les fonctions avec chacune des cibles dans la liste
                 * d'arguments (donc constantes), mais d'utiliser la combinations de Triggers pilote-opérateur pour caller
                 * la bonne "version" de la commande.
                 * 
                 * C'est pas le plus clean car nous avons énormément de répétition dans le code, mais ça fonctionne !
                  */

                ///////////////////Auto Corail (amener le corail au Récif)///////////////////
                
                //Lecture de la manette opérateur pour savoir la hauteur sur le Récif
                setHauteurOperateur();
                
                //Bouton A = Se rendre au corail automatiquement et lever le manipulateur à la bonne hauteur selon la manette opérateur
                autoCorailOperateur();

                ///////////////////Auto Station (récupérer un corail à la station)///////////////////

                //Bouton B = se rendre Station
                //AutoStation ne gère que la basepilotable
                //Avec les triggers gauche et droite pour se décaler sur la station
                //Permet de se rendre automatiquement vers la station la plus proche

                //Côté Cage
                manette.b().and(stationCageTrigger).and(stationGaucheTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationCageProche, basePilotable,
                                                ascenseur, poignet, corailManip));
                manette.b().and(stationCageTrigger).and(stationCentreTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationCageCentre, basePilotable,
                                                ascenseur, poignet, corailManip));
                manette.b().and(stationCageTrigger).and(stationDroiteTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationCageLoin, basePilotable,
                                                ascenseur, poignet, corailManip));

                //Côté Processeur
                manette.b().and(stationCageTrigger.negate()).and(stationGaucheTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationProcLoin, basePilotable,
                                                ascenseur, poignet, corailManip));
                manette.b().and(stationCageTrigger.negate()).and(stationCentreTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationProcCentre, basePilotable,
                                                ascenseur, poignet, corailManip));
                manette.b().and(stationCageTrigger.negate()).and(stationDroiteTrigger)
                                .whileTrue(new AutoStation(GamePositions.BlueCoralStationProcProche, basePilotable,
                                                ascenseur, poignet, corailManip));


                //Place le manipulateur à la bonne hauteur et gobe automatiquement quand on est proche d'une station
                procheStationTrigger.whileTrue(
                                new GoToHauteur(() -> Hauteur.station[0], () -> Hauteur.station[1], ascenseur, poignet)
                                                .alongWith(corailManip.goberCommand()).until(corailManip::isCorail));

                //////////////ALGUES////////////////////
                
                //Bouton X = aller chercher algue dans le récif selon la branche sélectionnée par l'opérateur
                autoAlgueOperateur();

                //Bouton Y = Auto algue au processeur
                manette.y().and(modeGrimpeurTrigger.negate())
                                .whileTrue(new AutoProcesseur(basePilotable, ascenseur, poignet));
               
                // Bumper gauche = Gober les algues au sol
                manette.leftBumper().whileTrue(algueManip.goberCommand().alongWith(new GoToHauteur(
                                () -> Hauteur.algueSol[0], () -> Hauteur.algueSol[1], ascenseur, poignet)));


                //////////////Sortir les pièces de jeu////////////////////
                // Bumper droit = Sortir les pieces de jeu
                manette.rightBumper().whileTrue(algueManip.sortirCommand().alongWith(corailManip.sortirCommand()));


                //////////////Mode Grimpeur////////////////////

                // Trigger gauche + droit = Activer/desactiver grimpeur
                grimpeurTrigger.onTrue(Commands.runOnce(() -> {
                        modeGrimpeur = !modeGrimpeur;
                }));

                //Contrôle du grimpeur
                modeGrimpeurTrigger.whileTrue(new ActiverGrimpeur(ascenseur, poignet)//Placer la pince en mode grimpeur
                                .andThen(new ControleGrimpeur(manette::getLeftTriggerAxis, manette::getRightTriggerAxis, ascenseur, poignet))//Trigger Gauche = monter et trigger droit = descendre
                                .alongWith(Commands.run(() -> del.rainbow(), del)));//Classique : Grimper = Rainbow

                // Bouton Start en mode grimpeur = Barrer le servo 
                manette.start().and(modeGrimpeurTrigger)
                                .toggleOnTrue(Commands.startEnd(ascenseur::barrer, ascenseur::debarrer));

                // Start = Homing dans le pit. Ne marche pas super bien
                //manette.start().onTrue(new PreparationPit(ascenseur, poignet));

                resetEncodeurTrigger.onTrue(Commands.runOnce(ascenseur::resetEncodersVortex));
        }



        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        private void setHauteurOperateur() {
                /* On actualise la cible dans une variable interne des deux sous-systèmes
                selon le bouton de la manette opérateur. On appelle ensuite cette variable
                pour connaitre la cible lorsqu'on active AutoCorail avec le bouton A.
                Ça fonctionne car la cible en hauteur et en angle sont des doubles, donc
                on a un DoubleSupplier*/
                operateur.button(BoutonOperateur.L1).onTrue(new SetHauteur(Hauteur.L1, ascenseur, poignet));
                operateur.button(BoutonOperateur.L2).onTrue(new SetHauteur(Hauteur.L2, ascenseur, poignet));
                operateur.button(BoutonOperateur.L3).onTrue(new SetHauteur(Hauteur.L3, ascenseur, poignet));
                operateur.button(BoutonOperateur.L4).onTrue(new SetHauteur(Hauteur.L4, ascenseur, poignet));

        }

        
        private void autoCorailOperateur() {
                /*Toutes les versions de autoCorail. Nécessaire car il n'y a pas de "Pose2dSupplier"
                DescenteAutomatique sert à protéger la pince du récif lors de sa descente
                Permet aussi de Hold la pince dans les airs en libérant la basepilotable pour permettre au pilote de s'ajuster*/
                manetteA.and(operateur.button(BoutonOperateur.A))
                                .whileTrue(new AutoCorail(
                                                Branche.A, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteA.and(operateur.button(BoutonOperateur.B))
                                .whileTrue(new AutoCorail(
                                                Branche.B, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteA.and(operateur.button(BoutonOperateur.C))
                                .whileTrue(new AutoCorail(
                                                Branche.C, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteA.and(operateur.button(BoutonOperateur.D))
                                .whileTrue(new AutoCorail(
                                                Branche.D, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteA.and(operateur.button(BoutonOperateur.E))
                                .whileTrue(new AutoCorail(
                                                Branche.E, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteA.and(operateur.button(BoutonOperateur.F))
                                .whileTrue(new AutoCorail(
                                                Branche.F, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteA.and(operateur.button(BoutonOperateur.G))
                                .whileTrue(new AutoCorail(
                                                Branche.G, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteA.and(operateur.button(BoutonOperateur.H))
                                .whileTrue(new AutoCorail(
                                                Branche.H, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteA.and(operateur.button(BoutonOperateur.I))
                                .whileTrue(new AutoCorail(
                                                Branche.I, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteA.and(operateur.button(BoutonOperateur.J))
                                .whileTrue(new AutoCorail(
                                                Branche.J, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteA.and(operateur.button(BoutonOperateur.K))
                                .whileTrue(new AutoCorail(
                                                Branche.K, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteA.and(operateur.button(BoutonOperateur.L))
                                .whileTrue(new AutoCorail(
                                                Branche.L, basePilotable, ascenseur, poignet))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

        }

        private void autoAlgueOperateur() {
        //Voir les commentaires dans le bloc de fonctions ci-haut
                manetteX.and(operateur.button(BoutonOperateur.A).or(operateur.button(BoutonOperateur.B)))
                                .whileTrue(new AutoAlgue(
                                                Algue.AB, Hauteur.algueHaut, ascenseur, poignet, basePilotable,
                                                algueManip))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteX.and(operateur.button(BoutonOperateur.C).or(operateur.button(BoutonOperateur.D)))
                                .whileTrue(new AutoAlgue(
                                                Algue.CD, Hauteur.algueBas, ascenseur, poignet, basePilotable,
                                                algueManip))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteX.and(operateur.button(BoutonOperateur.E).or(operateur.button(BoutonOperateur.F)))
                                .whileTrue(new AutoAlgue(
                                                Algue.EF, Hauteur.algueHaut, ascenseur, poignet, basePilotable,
                                                algueManip))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteX.and(operateur.button(BoutonOperateur.G).or(operateur.button(BoutonOperateur.H)))
                                .whileTrue(new AutoAlgue(
                                                Algue.GH, Hauteur.algueBas, ascenseur, poignet, basePilotable,
                                                algueManip))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteX.and(operateur.button(BoutonOperateur.I).or(operateur.button(BoutonOperateur.J)))
                                .whileTrue(new AutoAlgue(
                                                Algue.IJ, Hauteur.algueHaut, ascenseur, poignet, basePilotable,
                                                algueManip))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));

                manetteX.and(operateur.button(BoutonOperateur.K).or(operateur.button(BoutonOperateur.L)))
                                .whileTrue(new AutoAlgue(
                                                Algue.KL, Hauteur.algueBas, ascenseur, poignet, basePilotable,
                                                algueManip))
                                .onFalse((new DescenteAutomatique(basePilotable, ascenseur, poignet)));
        }

}

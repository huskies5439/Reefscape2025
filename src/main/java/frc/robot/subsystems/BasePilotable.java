// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.GamePositions;

public class BasePilotable extends SubsystemBase {

	// Créer les moteurs swerves
	private MAXSwerveModule avantGauche = new MAXSwerveModule(1, 2, -90);

	private MAXSwerveModule avantDroite = new MAXSwerveModule(3, 4, 0);

	private MAXSwerveModule arriereGauche = new MAXSwerveModule(5, 6, 180);

	private MAXSwerveModule arriereDroite = new MAXSwerveModule(7, 8, 90);

	// Le gyroscope
	private Pigeon2 gyro = new Pigeon2(0);


	/* Setpoint genetator est une fonction de PathPlanner qui permet de valider
	 * si les vitesses demandées en téléop respectent les contraintes mécaniques
	 * du robot telles que définies dans les App Settings de PathPlanner
	 * C'est expérimental en 2025, donc valider les changements pour l'an prochain */
	
	private final SwerveSetpointGenerator setpointGenerator;
	private SwerveSetpoint previousSetpoint;

	// Initialisation PoseEstimator
	SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
			Constants.kDriveKinematics,
			Rotation2d.fromDegrees(getAngle()),
			new SwerveModulePosition[] { avantGauche.getPosition(),
					avantDroite.getPosition(), arriereGauche.getPosition(),
					arriereDroite.getPosition() },
			Pose2d.kZero);

	Field2d field2d = new Field2d();


	public BasePilotable() {

		// Reset initial
		resetGyro();
		resetEncoders();
		resetOdometry(new Pose2d());

		// aller chercher la configuration du robot dans Pathplanner
		RobotConfig robotConfig = null;
		try {

			robotConfig = RobotConfig.fromGUISettings();

		} catch (Exception e) {
			e.printStackTrace();
		}
		// configuration Pathplanner
		//Voir la documentation, ça va peut-être changer en 2026
		AutoBuilder.configure(
				this::getPose,
				this::resetOdometry,
				this::getChassisSpeeds,
				(speeds, feedforward) -> conduireChassis(speeds),
				new PPHolonomicDriveController(
						new PIDConstants(2, 0, 0), // valeur stupide de 12 a Montréal ; ne plus faire l'Erreur S.V.P
						new PIDConstants(5, 0, 0)), 
				robotConfig,
				this::isRedAlliance,
				this);
		
		//Configuration Setpoint Generator
		setpointGenerator = new SwerveSetpointGenerator(
				robotConfig,
				Units.rotationsToRadians(3.94)// Valeur selon la freespeed du neo 550
		);

		resetSetpoint();

	}

	@Override
	public void periodic() {
		// Update du Pose Estimator
		poseEstimator.update(
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(),
						avantDroite.getPosition(), arriereGauche.getPosition(), arriereDroite.getPosition() });

		//Update du Field2d
		field2d.setRobotPose(getPose());
		SmartDashboard.putData("Field", field2d);

		//SmartDashboard.putBoolean("redalliance", isRedAlliance());

		SmartDashboard.putNumber("Angle Gyro", getAngle());

		SmartDashboard.putNumber("Pose Estimator X : ", getPose().getX());
		SmartDashboard.putNumber("Pose Estimator Y : ", getPose().getY());
		SmartDashboard.putNumber(
				"Pose Estimator Theta : ",
				getPose().getRotation().getDegrees());

		SmartDashboard.putNumber("match time",DriverStation.getMatchTime());
		//Fonctions limelight
		//Première année que ça fonctionne comme ça directement dans le sous-système de BasePilotable
		//Valider que cela ne change pas l'an prochain
		setLimelightRobotOrientation();
		addVisionPosition("limelight-haut");
		addVisionPosition("limelight-bas");
	}

	/// ////// MÉTHODE DONNANT DES CONSIGNES À CHAQUE MODULE

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		//La librairie de REV utilise la fonction .desaturate ici.
		//Attention, ils utilisent le maxChassisSpeed au lieu du maxVitesseModule
		//SetPointGenerator ôte la nécessiter de désaturer
		avantGauche.setDesiredState(desiredStates[0]);
		avantDroite.setDesiredState(desiredStates[1]);
		arriereGauche.setDesiredState(desiredStates[2]);
		arriereDroite.setDesiredState(desiredStates[3]);
	}

	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] { avantGauche.getState(),
				avantDroite.getState(), arriereGauche.getState(),
				arriereDroite.getState() };
	}

	/// ///// TÉLÉOP

	public void resetSetpoint(){
		//Il faut le caller à chaque fois que l'on retourne à la commande de conduite téléop
		//Sinon les premiers mouvements du robot dépendent de la conduite avant la commande de pathfinding
		previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
	}
	

	public void conduire(
			double xSpeed,
			double ySpeed,
			double rot,
			boolean fieldRelative,
			boolean squared) {

		double deadband = 0.05;
		// appliquer une deadband sur les joysticks et corriger la direction
		xSpeed = -MathUtil.applyDeadband(xSpeed, deadband);
		ySpeed = -MathUtil.applyDeadband(ySpeed, deadband);
		rot = -MathUtil.applyDeadband(rot, deadband);

		if (squared) {// Mettre les joysticks "au carré" pour adoucir les
			// déplacements
			xSpeed = xSpeed * Math.abs(xSpeed);
			ySpeed = ySpeed * Math.abs(ySpeed);
			rot = rot * Math.abs(rot);
		}

		// Convert the commanded speeds into the correct units for the
		// drivetrain
		double xSpeedDelivered = xSpeed * Constants.maxVitesseLineaire;
		double ySpeedDelivered = ySpeed * Constants.maxVitesseLineaire;
		double rotDelivered = rot * Constants.maxVitesseRotation;

		// inversion du field oriented selon l'alliance
		double invert = 1;
		if (isRedAlliance()) {
			invert = -1; // on inverse le déplacement du robot
		}

		//Ajuster pour field relative
		//L'inversion selon l'alliance seulement nécessaire en x et y en field oriented
		//Façon vraiment plus clean de gérer ça qu'en 2024
		ChassisSpeeds speeds = fieldRelative ? 
				ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered * invert, ySpeedDelivered * invert,
				rotDelivered, getPose().getRotation())
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

		//Actualiser le setpoint generator pour corriger le mouvement si nécessaire
		previousSetpoint = setpointGenerator.generateSetpoint(
				previousSetpoint,
				speeds,
				0.02);

		//Envoyer les consignes aux 4 modules
		setModuleStates(previousSetpoint.moduleStates());
	}

	public void stop() {
		conduire(0, 0, 0, false, false);

	}

	// Sets the wheels into an X formation to prevent movement.
	public void setX() {
		avantGauche.setDesiredState(new SwerveModuleState(
				0,
				Rotation2d.fromDegrees(
						45)));
		avantDroite.setDesiredState(new SwerveModuleState(
				0,
				Rotation2d.fromDegrees(
						-45)));
		arriereGauche.setDesiredState(new SwerveModuleState(
				0,
				Rotation2d.fromDegrees(
						-45)));
		arriereDroite.setDesiredState(new SwerveModuleState(
				0,
				Rotation2d.fromDegrees(
						45)));
	}

	/// ////// Pose estimator
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public void resetOdometry(Pose2d pose) {// pose est à la pose où reset, c'est typiquement l'origine du terrain
		poseEstimator.resetPosition(
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(),
						avantDroite.getPosition(), arriereGauche.getPosition(), arriereDroite.getPosition() },
				pose);
	}

	/// ///////////////// limelight
	/// On utilise MegaTag 2 cette année
	/// Algorithme puissant, mais il faut connaître l'angle du robot
	/// Donc TOUJOURS ouvrir le robot/compiler en pointant 0°
	/// Voir la documentation limelight, il se peut que ça change l'an prochain !
	public void setLimelightRobotOrientation() {
		LimelightHelpers.SetRobotOrientation(
				"limelight-haut",
				poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
				0,
				0,
				0,
				0,
				0);
		LimelightHelpers.SetRobotOrientation(
				"limelight-bas",
				poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
				0,
				0,
				0,
				0,
				0);
	}

	public void addVisionPosition(String nomComplet) {

		// parametre limelight
		poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
				0.7,
				0.7,
				9999999));

		LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
				nomComplet);
		boolean doRejectUpdate = false;
		if (poseEstimate == null) {
			return;
		}

		if (Math.abs(getRate()) > 720) {
			doRejectUpdate = true;
		}
		if (poseEstimate.tagCount == 0) {
			doRejectUpdate = true;
		}
		SmartDashboard.putBoolean(nomComplet, !doRejectUpdate);
		if (!doRejectUpdate) {
			poseEstimator.addVisionMeasurement(
					poseEstimate.pose,
					poseEstimate.timestampSeconds);
		}
	}

	////////////// Encodeurs
	// Pas besoin de méthode pour obtenir la position des encodeurs, tout ça
	// passe directement par la pose2D du robot
	public void resetEncoders() {
		avantGauche.resetEncoders();
		arriereGauche.resetEncoders();
		avantDroite.resetEncoders();
		arriereDroite.resetEncoders();
	}

	/////////////// GYRO
	public double getAngle() {
		return gyro.getYaw().getValueAsDouble();
	}

	public double getRate() {
		return gyro.getAngularVelocityZWorld().getValueAsDouble();
	}

	public void resetGyro() {
		gyro.setYaw(0);
	}

	/// ///////////// Path Planner
	public ChassisSpeeds getChassisSpeeds() {
		return Constants.kDriveKinematics.toChassisSpeeds(
				avantDroite.getState(),
				avantGauche.getState(),
				arriereDroite.getState(),
				arriereGauche.getState());
	}

	public void conduireChassis(ChassisSpeeds chassisSpeeds) {
		// Ramene la vitesse en intervale de 20 ms
		ChassisSpeeds targetSpeed = ChassisSpeeds.discretize(
				chassisSpeeds,
				0.02);

		SwerveModuleState[] swerveModuleState = Constants.kDriveKinematics.toSwerveModuleStates(
				targetSpeed);
		setModuleStates(swerveModuleState);
	}




	/////////////// On the fly

	public Command followPath(Pose2d cible) {

		PathConstraints constraints = new PathConstraints(
				1.75,
				1.0,
				Math.toRadians(360),
				Math.toRadians(360)); 
		//Hyper Important : Il faut mettre la méthode "flipped" pour ajuster pour RedAlliance
		//Fonction pas mentionnée dans la doc !!
		return AutoBuilder.pathfindToPoseFlipped(cible, constraints, 0.0);

	}

	//////////////isProche permet de vérifier la position du robot
	/// Seule fois où on a besoin des Pose2D de l'alliance rouge car on ne passe pas par PathPlanner
	public boolean isProche(Pose2d cible, double distanceMin) {
		return getPose().getTranslation().getDistance(cible.getTranslation()) < distanceMin;
	}

	public boolean isProcheRecif() {//Pour la monter
		return isProche(
				isRedAlliance() ? GamePositions.RedCentreRecif : GamePositions.BlueCentreRecif,
				4);
	}

	public boolean isLoinRecif() {//Pour la descente
		return !isProche(
				isRedAlliance() ? GamePositions.RedCentreRecif : GamePositions.BlueCentreRecif,
				1.5);
	}

	public boolean isProcheProcesseur() {
		return isProche(
				isRedAlliance() ? GamePositions.RedProcesseur : GamePositions.BlueProcesseur,
				2);
	}

	public boolean isProcheStationCage() {
		return isProche(
				isRedAlliance() ? GamePositions.RedCoralStationCage : GamePositions.BlueCoralStationCageCentre,
				1);
	}

	public boolean isProcheStationProcesseur() {
		return isProche(
				isRedAlliance() ? GamePositions.RedCoralStationProc : GamePositions.BlueCoralStationProcCentre,
				1);
	}

	public boolean isStationCage() {
		return isRedAlliance() ^ getPose().getY() >= 4;
	}


	//Vérifier l'alliance. Il faut le caller en tout temps car l'alliance est initialiser après le boot du robot
	public boolean isRedAlliance() {
		Optional<Alliance> ally = DriverStation.getAlliance();
		if (ally.isPresent()) {
			return ally.get() == Alliance.Red;

		} else {
			return false;
		}
	}

}

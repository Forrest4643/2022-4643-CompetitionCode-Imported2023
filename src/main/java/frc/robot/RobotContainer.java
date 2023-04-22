// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

        private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
        private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
        private final Sensors m_sensors = new Sensors();
        private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
        private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
        private final ShooterPIDSubsystem m_shooterPIDsubsystem = new ShooterPIDSubsystem();
        private final HoodPIDSubsystem m_hoodPIDsubsystem = new HoodPIDSubsystem();
        private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
        private final ClimberSubsystem m_climbersubsystem = new ClimberSubsystem();
        private final TurretPIDSubsystem m_turretPIDsubsystem = new TurretPIDSubsystem(m_visionSubsystem, m_sensors);
        private final XboxController m_driveController = new XboxController(0);
        private final XboxController m_operateController = new XboxController(1);

        private final LookForTarget m_lookfortarget = new LookForTarget(m_turretPIDsubsystem);
        private final TrackTarget m_tracktarget = new TrackTarget(m_turretPIDsubsystem);
        private final TurretPosition m_turretposition = new TurretPosition(m_turretPIDsubsystem, -160);

        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                m_driveSubsystem.setDefaultCommand(
                                new StickDrive(m_driveSubsystem, m_driveController, m_turretPIDsubsystem));

        }

        private void configureButtonBindings() {

                //climber up
                new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(m_climbersubsystem::up))
                                .onFalse(new InstantCommand(m_climbersubsystem::idle));

                //climber down
                new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value).onTrue(new InstantCommand(m_climbersubsystem::down))
                                .onFalse(new InstantCommand(m_climbersubsystem::idle));

                //Front intake
                new JoystickButton(m_operateController, XboxController.Button.kY.value)
                                .onTrue(new InstantCommand(m_pneumaticsSubsystem::frontIntakeOpen))
                                .onFalse(new InstantCommand(m_pneumaticsSubsystem::frontIntakeClosed));

                //Rear intake
                new JoystickButton(m_operateController, XboxController.Button.kA.value)
                                .onTrue(new InstantCommand(m_pneumaticsSubsystem::rearIntakeOpen))
                                .onFalse(new InstantCommand(m_pneumaticsSubsystem::rearIntakeClosed));

                //shooter+hood activate
                //**Commented for demos */

                //new JoystickButton(m_operateController, XboxController.Button.kX.value).whileTrue(
                //new AutoAim(m_hoodPIDsubsystem, m_visionSubsystem, m_shooterPIDsubsystem));

                //TEMP FOR DEMOS
                new JoystickButton(m_operateController, XboxController.Button.kX.value).whileTrue(new InstantCommand(m_shooterPIDsubsystem::fullSendShooter)
                        .alongWith(new InstantCommand(m_hoodPIDsubsystem::hoodOpen))).onFalse(new InstantCommand(m_shooterPIDsubsystem::idleShooter)
                        .alongWith(new InstantCommand(m_hoodPIDsubsystem::hoodClosed)));

                //activate turret
                new JoystickButton(m_operateController, XboxController.Button.kB.value)
                                .toggleOnTrue(new ActivateTurret(m_tracktarget, m_lookfortarget, m_visionSubsystem,
                                                                m_turretPIDsubsystem));

                //hub shot
                new JoystickButton(m_operateController, XboxController.Button.kRightStick.value).toggleOnTrue(
                                new HUB(m_turretposition, m_shooterPIDsubsystem, m_hoodPIDsubsystem,
                                                m_turretPIDsubsystem));

                //shooter reverse
                new JoystickButton(m_operateController, XboxController.Button.kLeftStick.value)
                                .onTrue(new InstantCommand(m_shooterPIDsubsystem::backDrive))
                                .onFalse(new InstantCommand(m_shooterPIDsubsystem::idleShooter));

        }

        public Command getAutonomousCommand() {

                // SmartDashboard.putBoolean("autonStart", true);
                // return new DriveDistance(m_driveSubsystem, DriveConstants.autoDist);

                // return new SequentialCommandGroup(
                // new ParallelDeadlineGroup(
                // new WaitCommand(3),
                // new HUB(m_turretposition, m_shooterPIDsubsystem, m_hoodPIDsubsystem,
                // m_turretPIDsubsystem),
                // new SequentialCommandGroup(
                // new WaitCommand(1.75),
                // new InstantCommand(m_indexerSubsystem::wheelsOn),
                // new WaitCommand(1),
                // new InstantCommand(m_indexerSubsystem::wheelsOff)
                // )
                // ),
                // new DriveDistance(m_driveSubsystem, DriveConstants.autoDist)
                // );

                return null;

        }

        public void teleInit() {
                m_intakeSubsystem.setDefaultCommand(
                                new AutoIndex(m_intakeSubsystem, m_indexerSubsystem, m_pneumaticsSubsystem, m_sensors,
                                                m_operateController));
        }

}

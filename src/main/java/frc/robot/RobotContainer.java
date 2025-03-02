// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import javax.xml.transform.TransformerException;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.geometry.Transform2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final SendableChooser<Command> autoChooser;


  

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  XboxController m_OperatorController = new XboxController(OIConstants.kOperatorControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    NamedCommands.registerCommand("troughShot", new RunCommand( () 
    -> m_shooter.trough(1), m_shooter).withTimeout(0.5).withName("troughShot"));

    NamedCommands.registerCommand("stopTrough", new RunCommand( () 
    -> m_shooter.trough(0), m_shooter).withTimeout(.1).withName("stopTrough"));


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    m_shooter.setDefaultCommand(
      new RunCommand(
      () -> m_shooter.shoot(m_OperatorController.getRawAxis(3)),
      m_shooter)
    );

  
 m_elevator.setDefaultCommand(
  new RunCommand(
    () -> m_elevator.elevatorControl(0), m_elevator));

  

/*m_elevator.setDefaultCommand(
      new RunCommand(
        () -> m_elevator.elevatorControl(m_OperatorController.getLeftY()),
        m_elevator)
    );
    */
    configureButtonBindings();
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser("TEST");
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    new JoystickButton(m_driverController, 10)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.changeSpeedLow(),
            m_robotDrive));
    
    new JoystickButton(m_driverController, 9)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.changeSpeedHigh(),
              m_robotDrive));

    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
 
    new JoystickButton(m_driverController, 7)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));

    new JoystickButton(m_driverController, 13)
    .onTrue(new InstantCommand(
      () -> {
      final MedianFilter p1Filter = new MedianFilter(10);
      final MedianFilter p2Filter = new MedianFilter(10);
      final MedianFilter p3Filter = new MedianFilter(10);
      final MedianFilter p4Filter = new MedianFilter(10);
      new RunCommand(() -> {
        double[] LLPoseI = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        p1Filter.calculate(LLPoseI[0]);
        p2Filter.calculate(LLPoseI[1]);
        p3Filter.calculate(LLPoseI[2]);
        p4Filter.calculate(LLPoseI[3]);
      }
      , m_robotDrive).withTimeout(0.1).andThen(
      new InstantCommand(() -> {
      double[] LLPoseT = LimelightHelpers.getTargetPose_RobotSpace("limelight");
      double[] LLPose = {
        p1Filter.calculate(LLPoseT[0]),
        p2Filter.calculate(LLPoseT[1]),
        p3Filter.calculate(LLPoseT[2]),
        p4Filter.calculate(LLPoseT[3])
      };
      Pose2d LLPose2d = new Pose2d(LLPose[2], LLPose[0], Rotation2d.fromRadians(LLPose[3]));
      AutoBuilder.followPath(
        PathplannerUTILS.createLLPath(
          m_robotDrive.getPose(), 
          m_robotDrive.getPose().plus(new Transform2d(LLPose2d.getX()-0.2, LLPose2d.getY(), Rotation2d.fromDegrees(0))))
          ).schedule();
    })).schedule();
      
        }));

        new JoystickButton(m_driverController, 1)
        .onTrue(new RunCommand(
          () -> m_shooter.shoot(1), 
        m_shooter).withTimeout(0.5));

        new JoystickButton(m_driverController, 2)
        .onTrue(new RunCommand(
          () -> m_shooter.trough(1), 
        m_shooter).withTimeout(0.75));

       /*  new JoystickButton(m_driverController, 11)
        .onTrue(new ParallelRaceGroup(
          m_shooter.highShot(1), 
        m_shooter).withTimeout(0.75),m_elevator.level3Position(42), 
        m_elevator);
*/
new JoystickButton(m_driverController,11)
        .onTrue(new RunCommand(
          () -> m_shooter.highShot(1), 
        m_shooter).withTimeout(0.75));



        new JoystickButton(m_OperatorController, XboxController.Axis.kLeftY.value)
        .whileTrue(new RunCommand(
          () -> m_elevator.elevatorControl(m_OperatorController.getLeftY()),
          m_elevator));
 
          new JoystickButton(m_OperatorController, 6)
          .whileTrue(new RunCommand(
           () ->m_shooter.laserIntake(.25), 
          m_shooter) );
 
          new JoystickButton(m_OperatorController, 7)
          .whileTrue
          (new RunCommand(
            () -> m_elevator.elevatorControl(m_OperatorController.getLeftY()),
            m_elevator));
          


          new JoystickButton(m_OperatorController, 5)
          .whileTrue(new RunCommand(
           () ->m_shooter.reverse(.2), 
          m_shooter) );
 
          new Trigger(() -> m_OperatorController.getRawButton(2))
          .whileTrue(new RunCommand(() -> m_elevator.level1Position(8.2), 
          m_elevator)) ;

          new Trigger(() -> m_OperatorController.getRawButton(3))
          .whileTrue(new RunCommand(() -> m_elevator.level2Position(22), 
          m_elevator)) ;

          new Trigger(() -> m_OperatorController.getRawButton(1))
          .whileTrue(new RunCommand(() -> m_elevator.level0Position(2), 
          m_elevator)) ;

          new Trigger(() -> m_OperatorController.getRawButton(4))
          .whileTrue(new RunCommand(() -> m_elevator.level3Position(42), 
          m_elevator)) ;
          

          new Trigger(() -> m_OperatorController.getRawButton(8))
          .whileTrue(new RunCommand(() -> m_elevator.levelMaxPosition(45), 
          m_elevator)) ;
          }
        
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }




}



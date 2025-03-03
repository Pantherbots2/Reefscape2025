// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Robot extends TimedRobot {
  private final PS4Controller controller = new PS4Controller(0);
  private final Joystick drivercontroller = new Joystick(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private PWMSparkMax ClawMotor;
  public final double ClawOpenSpeed = 1;
  public final double ClawCloseSpeed = -1;
  private PWMSparkMax ElevatorMotor;
  public final double ElevatorUpSpeed = 1;
  public final double ElevatorDownSpeed = -1;
  private PWMSparkMax ClawRotationMotor;
  public final double ClawLiftSpeed = 1;
  public final double ClawLowerSpeed = -1;
  private PWMSparkMax CoralMotor;
  public final double CoralForwardSpeed = 1;
  public final double CoralBackwardSpeed = -1;
  private PWMSparkMax CoralPickupMotor;
  public final double CoralExtractionSpeed = 1;
  public final double CoralRetractionSpeed = -1;
  private PWMSparkMax CageMotor;
  public final double CageLiftSpeed = 1;
  public final double CageLowerSpeed = -1;
  public final double NoSpeed = 0;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);


    if (controller.getL1Button()) {
      ClawMotor.set(ClawOpenSpeed);
    }else if(controller.getL2Button()){
      ClawMotor.set(ClawCloseSpeed);
    } else{
      ClawMotor.set(NoSpeed);
    }
    if (controller.getL1Button()) {
      ElevatorMotor.set(ElevatorUpSpeed);
    }else if(controller.getL2Button()){
      ElevatorMotor.set(ElevatorDownSpeed);
    } else{
      ElevatorMotor.set(NoSpeed);
    }
    if (controller.getL1Button()) {
      ClawRotationMotor.set(ClawLiftSpeed);
    }else if(controller.getL2Button()){
      ClawRotationMotor.set(ClawLowerSpeed);
    } else{
      ClawRotationMotor.set(NoSpeed);
    }
    if (controller.getL1Button()) {
      CoralMotor.set(CoralForwardSpeed);
    }else if(controller.getL2Button()){
      CoralMotor.set(CoralBackwardSpeed);
    } else{
      CoralMotor.set(NoSpeed);
    }
    if (controller.getL1Button()) {
      CoralPickupMotor.set(CoralExtractionSpeed);
    }else if(controller.getL2Button()){
      CoralPickupMotor.set(CoralRetractionSpeed);
    } else{
      CoralPickupMotor.set(NoSpeed);
    }
    if (controller.getL1Button()) {
      CageMotor.set(CageLiftSpeed);
    }else if(controller.getL2Button()){
      CageMotor.set(CageLowerSpeed);
    } else{
      CageMotor.set(NoSpeed);
    }

    
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(drivercontroller.getRawAxis(0), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(drivercontroller.getRawAxis(1), 0.02))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(drivercontroller.getRawAxis(2), 0.02))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}

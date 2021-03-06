// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.subsystems.Rev3ColorSensor;

/**
 * This is Team 2530's Shooter class.
 */
public class Shooter extends SubsystemBase {
  private static WPI_TalonFX shooterMotor = new WPI_TalonFX(Constants.SHOOTER_MOTOR_PORT);
  XboxController xbox;
  public static double shooterSpeedWithTriggerChange = 0.55;
  private static double triggerChange = 0.0075;

  private static NetworkTableEntry shooterPowerDial = Shuffleboard.getTab("Driver Dashboard")
      .add("Shooter Speed", shooterSpeedWithTriggerChange * 100)
      .withWidget(BuiltInWidgets.kDial)
      .withSize(2, 1)
      .withPosition(3, 0)
      .withProperties(Map.of("min", Constants.minShooterSpeed * 100, "max", Constants.maxShooterSpeed * 100))
      .getEntry();

  /** Creates a new {@link Intake}. */
  public Shooter(XboxController xbox) {
    this.xbox = xbox;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // takes the desired shooter speed and puts it into a variable to be
    // used in Robot Container's shooter
    if (DriverStation.isTeleopEnabled()) {
      shooterSpeedWithTriggerChange = changeShooterSpeed(shooterSpeedWithTriggerChange);
    }
  }

  /**
   * Sets the speed and direction of the shooter motor.
   * 
   * @param speed Any value from -1.0 to 1.0.
   */
  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  /**
   * Uses the right trigger to run the shooter motor up to the maxSpeed parameter
   * 
   * @param maxSpeed The maximum speed you would like the shooter motor to run at
   */
  public void setShooterSpeedTrigger(double maxSpeed) {
    if (xbox.getRawAxis(3) > 0.1) {
      shooterMotor.set(xbox.getRawAxis(3) * maxSpeed);
    } else if (xbox.getRawAxis(3) < 0.1 && !xbox.getRawButton(3)) {
      shooterMotor.set(0.0);
    }
  }

  /**
   * takes the current intake speed and changes it based on the trigger inputs
   * 
   * @param currentShooterSpeed what the current set shooter speed is
   * @return motor intake speed that is gained from using the triggers
   */
  public double changeShooterSpeed(double currentShooterSpeed) {
    if (xbox.getRawAxis(3) > 0.1) {
      currentShooterSpeed += Shooter.triggerChange;
    } else if (xbox.getRawAxis(2) > 0.1) {
      currentShooterSpeed -= Shooter.triggerChange;
    }

    // Dont change these no matter how backwards they seem
    currentShooterSpeed = Math.min(currentShooterSpeed, Constants.maxShooterSpeed);
    currentShooterSpeed = Math.max(currentShooterSpeed, Constants.minShooterSpeed);

    shooterPowerDial.setValue(currentShooterSpeed * 100);
    return currentShooterSpeed;
  }

  /**
   * Calculates whether or not the shooter is running at the minimum speed
   * threshold to shoot a ball.
   */
  public static boolean meetsSpeedThreshold() {
    double threshold = shooterSpeedWithTriggerChange * (Constants.shooterSpeedThreshold / 100);
    double actual = shooterMotor.getMotorOutputPercent();
    return actual >= threshold;
  }
}
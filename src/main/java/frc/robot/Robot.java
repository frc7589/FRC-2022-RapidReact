// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser < String > m_chooser = new SendableChooser < > ();

  // private XboxController con1;
  private XboxController con2;

  private WPI_VictorSPX victorfrontLeft;
  private WPI_VictorSPX victorbackLeft;
  private WPI_VictorSPX victorfrontRight;
  private WPI_VictorSPX victorbackRight;
  private MecanumDrive robotDrive;
  private MotorControllerGroup leftdrive;
  private MotorControllerGroup rightdrive;
  
  private WPI_VictorSPX transportDown;
  private WPI_VictorSPX transportUp;

  private WPI_VictorSPX inTakeOut;
  private WPI_VictorSPX inTakeOn;

  private CANSparkMax shootUp;
  private CANSparkMax shootDown;

  private SparkMaxPIDController PID_shootUp;
  private SparkMaxPIDController PID_shootDown;

  private RelativeEncoder encoder_shootup; //min 速度
  private RelativeEncoder encoder_shootdown;

  private RobotController robotController;

  int timer = 0;
  int speedChannel = 0;
  double speed[] = {     
    0.2,
    0.4,
    0.6,
    0.7,
    1
  };
  double shootUpSpeed[] = {1000,1500,2000};
  double shootDownSpeed[] = {1000,1500,2000};
  int shootUpSpeedChannel = 0;
  int shootDownSpeedChannel = 0;

  int pov0 = 0;
  int pov90 = 0;
  int pov180 = 0;
  int pov270 = 0;

  double Voltage;

  double kP_shoot = 0.0000000001;
  double kI_shoot = 0.0000000001;
  double kD_shoot = 0.0000001;
  double kF_shoot = 0.00018;

  boolean con2_AButtonPressed = true;
  boolean con2_BButtonPressed = true;
  boolean con2_XButtonPressed = true;
  boolean con2_YButtonPressed = true;
  boolean con2_StartButtonPressed = true;
  boolean con2_BackButtonPressed = true;
  boolean con2_RightStickButtonPressed = true;
  boolean con2_LeftStickButtonPressed = true;
  boolean con2_RightBumperPressed = true;
  boolean con2_LeftBumperPressed = true;

//////////////////////////////

  private CANSparkMax NEO_climb3;
  private CANSparkMax NEO_climb2;
  private CANSparkMax NEO_angle;

  private RelativeEncoder NEOencoder_climb3;
  private RelativeEncoder NEOencoder_climb2;
  private RelativeEncoder NEOencoder_angle;

  private SparkMaxPIDController NeoPID_climb3;
  private SparkMaxPIDController NeoPID_climb2;
  private SparkMaxPIDController NeoPID_angle;

  private WPI_VictorSPX PGback;
  private WPI_VictorSPX PGfront;

  double NEOpos1;
  double NEOpos2;
  double NEOpos3;

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    SmartDashboard.putNumber("kP", kP_shoot);
    SmartDashboard.putNumber("kI", kI_shoot);
    SmartDashboard.putNumber("kD", kD_shoot);
    SmartDashboard.putNumber("kF", kF_shoot);

    SmartDashboard.putNumber("kP_angle", 0.1);


    CameraServer.startAutomaticCapture(0);

    con2 = new XboxController(0);

    transportDown = new WPI_VictorSPX(11);
    transportUp = new WPI_VictorSPX(7);

    inTakeOut = new WPI_VictorSPX(13);
    inTakeOn = new WPI_VictorSPX(5);
    
    PGback = new WPI_VictorSPX(9);
    PGfront = new WPI_VictorSPX(4);

    victorfrontLeft = new WPI_VictorSPX(0);
    victorbackLeft = new WPI_VictorSPX(2);
    victorfrontRight = new WPI_VictorSPX(10);
    victorbackRight = new WPI_VictorSPX(6);

    leftdrive = new MotorControllerGroup(victorfrontLeft, victorbackLeft);
    rightdrive = new MotorControllerGroup(victorfrontRight, victorbackRight);
  
    robotDrive = new MecanumDrive(victorfrontLeft, victorbackLeft, victorfrontRight, victorbackRight);

    shootUp = new CANSparkMax(2, MotorType.kBrushless);
    shootDown = new CANSparkMax(1, MotorType.kBrushless);

    NEO_climb3 = new CANSparkMax(4,MotorType.kBrushless); //climb3
    NEO_climb2 = new CANSparkMax(5,MotorType.kBrushless); //climb2
    NEO_angle = new CANSparkMax(7,MotorType.kBrushless);  //angle
    
    PID_shootUp = shootUp.getPIDController();
    PID_shootDown = shootDown.getPIDController();

    NeoPID_climb3 = NEO_climb3.getPIDController();
    NeoPID_climb2 = NEO_climb2.getPIDController();
    NeoPID_angle = NEO_angle.getPIDController();

    encoder_shootup = shootUp.getEncoder();
    encoder_shootdown = shootDown.getEncoder();

    NEOencoder_climb3 = NEO_climb3.getEncoder();
    NEOencoder_climb2 = NEO_climb2.getEncoder();
    NEOencoder_angle = NEO_angle.getEncoder();

    PID_shootUp.setP(kP_shoot);
    PID_shootUp.setI(kI_shoot);
    PID_shootUp.setD(kD_shoot);
    PID_shootUp.setIZone(0);
    PID_shootUp.setFF(kF_shoot);
    PID_shootUp.setOutputRange(-1, 1);

    PID_shootDown.setP(kP_shoot);
    PID_shootDown.setI(kI_shoot);
    PID_shootDown.setD(kD_shoot);
    PID_shootDown.setIZone(0);
    PID_shootDown.setFF(kF_shoot);
    PID_shootDown.setOutputRange(-1, 1);

    NeoPID_climb3.setP(.005);
    NeoPID_climb3.setI(.0);
    NeoPID_climb3.setD(.0);
    NeoPID_climb3.setIZone(0);
    NeoPID_climb3.setFF(0);
    NeoPID_climb3.setOutputRange(-1, 1);
    NeoPID_climb3.setReference(0, ControlType.kVelocity);

    NeoPID_climb2.setP(.005);
    NeoPID_climb2.setI(.0);
    NeoPID_climb2.setD(.0);
    NeoPID_climb2.setIZone(0);
    NeoPID_climb2.setFF(0);
    NeoPID_climb2.setOutputRange(-1, 1);
    NeoPID_climb2.setReference(0, ControlType.kVelocity);

    NeoPID_angle.setP(.1); 
    NeoPID_angle.setI(.0);
    NeoPID_angle.setD(.0);
    NeoPID_angle.setIZone(0);
    NeoPID_angle.setFF(0);
    NeoPID_angle.setOutputRange(-1, 1);
    NeoPID_angle.setReference(0, ControlType.kVelocity);
  
  }

  @Override
  public void robotPeriodic() {

    PID_shootUp.setP(SmartDashboard.getNumber("kP", kP_shoot));
    PID_shootUp.setI(SmartDashboard.getNumber("kI", kI_shoot));          
    PID_shootUp.setD(SmartDashboard.getNumber("kD", kD_shoot));
    PID_shootUp.setFF(SmartDashboard.getNumber("kF", kF_shoot));

    PID_shootDown.setP(SmartDashboard.getNumber("kP", kP_shoot));
    PID_shootDown.setI(SmartDashboard.getNumber("kI", kI_shoot));
    PID_shootDown.setD(SmartDashboard.getNumber("kD", kD_shoot));
    PID_shootDown.setFF(SmartDashboard.getNumber("kF", kF_shoot));

    NeoPID_angle.setP(SmartDashboard.getNumber("kP_angle", 0.1));

    SmartDashboard.putNumber("shootupSpeed", shootUpSpeed[shootUpSpeedChannel]);
    SmartDashboard.putNumber("shootdownSpeed", shootUpSpeed[shootDownSpeedChannel]);

    Voltage = robotController.getBatteryVoltage();
  }

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        break;
      case kDefaultAuto:
      default:
        break;
    }

    timer++;
    if(timer < 150){
      PID_shootUp.setReference(800, ControlType.kVelocity);
      PID_shootDown.setReference(800, ControlType.kVelocity);
    }
    else if(timer < 200){
      PID_shootUp.setReference(800, ControlType.kVelocity);
      PID_shootDown.setReference(800, ControlType.kVelocity);
      transportUp.set(0.5);
    }
    else if(timer < 225){
      leftdrive.set(0.4);
      rightdrive.set(0.4);
    }
    else if(timer < 300){
      inTakeOut.set(0.5);
    }
    else if(timer < 400){
      PID_shootUp.setReference(1000, ControlType.kVelocity);
      PID_shootDown.setReference(1000, ControlType.kVelocity);
    }
    else if(timer < 475){
      inTakeOn.set(0.6);
      transportDown.set(0.4);
      PID_shootUp.setReference(1000, ControlType.kVelocity);
      PID_shootDown.setReference(1000, ControlType.kVelocity);
      transportUp.set(0.5);
      leftdrive.set(0.3);
      rightdrive.set(-0.3);
    }
    else if(timer < 525){
      PID_shootUp.setReference(1000, ControlType.kVelocity);
      PID_shootDown.setReference(1000, ControlType.kVelocity);
      transportUp.set(0.5);
    }
    else{
      inTakeOut.set(0);
      inTakeOn.set(0);
      transportDown.set(0);
      PID_shootUp.setReference(0, ControlType.kVelocity);
      PID_shootDown.setReference(0, ControlType.kVelocity);
      transportUp.set(0);
      leftdrive.set(0);
      rightdrive.set(0);
    }
    // if(timer < 50){
    //   leftdrive.set(-0.6);
    //   rightdrive.set(0.6);
    // }

    // else if(timer < 150){
    //   PID_shootUp.setReference(1200, ControlType.kVelocity);
    //   PID_shootDown.setReference(1500 , ControlType.kVelocity);
    // }
    // else if(timer < 250){
    //   transportUp.set(0.5);
    // }
    // else if(timer < 325){
    //   leftdrive.set(-0.4);
    //   rightdrive.set(0.4);
    // }
  }

  @Override
  public void teleopInit() {
    NEOpos1 = NEOencoder_climb3.getPosition();
    NEOpos2 = NEOencoder_climb2.getPosition();
    NEOpos3 = NEOencoder_angle.getPosition();   
  }
  
  @Override
  public void teleopPeriodic() {
//climb3
      if(con2.getAButtonReleased() || con2.getBButtonReleased()){
        NEO_climb3.set(0);
        NEOpos1 = NEOencoder_climb3.getPosition();
      }
  
      if(con2.getAButton()){
        NEO_climb3.set(0.3);
      }else if(con2.getBButton()){
        NEO_climb3.set(-0.3);
      }else{
        NeoPID_climb3.setReference(NEOpos1,ControlType.kPosition);
      }
  //climb2
      if(con2.getXButtonReleased()||con2.getYButtonReleased()){
        NEO_climb2.set(0);
        NEOpos2 = NEOencoder_climb2.getPosition();
      }
  
      if(con2.getXButton()){
        NEO_climb2.set(0.3);  //僅
      }else if(con2.getYButton()){
        NEO_climb2.set(-0.3); //松
      }else{
        NeoPID_climb2.setReference(NEOpos2,ControlType.kPosition);
      }
////angle////
      if(con2.getStartButtonReleased() || con2.getBackButtonReleased()){
        NEO_angle.set(0);
        NEOpos3 = NEOencoder_angle.getPosition();
      }
  
      if(con2.getBackButton()){
        NEO_angle.set(0.3);  //僅
      }else if(con2.getStartButton()){
        NEO_angle.set(-0.3); //松
      }else{
        NeoPID_angle.setReference(NEOpos3,ControlType.kPosition);
      }


      if(con2.getPOV() == 0 || con2.getPOV() == 180){
        NEO_angle.set(0);
        NEOpos3 = NEOencoder_angle.getPosition();
      }

      if(con2.getPOV() == 0 ){
        NEO_angle.set(0.3);  //僅
      }else if(con2.getPOV() == 180){
        NEO_angle.set(-0.3); //松
      }else{
        NeoPID_angle.setReference(NEOpos3,ControlType.kPosition);
      }

///////back PG
      if(con2.getLeftTriggerAxis() >= 0.2){
        PGback.set(0.9);
      }else if(con2.getRightTriggerAxis() >= 0.2){
        PGback.set(-0.9);
      }else{
        PGback.set(0);
      }
 ////front PG
      if(con2.getLeftBumper()){
        PGfront.set(0.9);
      }else if(con2.getRightBumper()){
        PGfront.set(-0.9);
      }else{
        PGfront.set(0);
      }

/////////gear
      if (con2.getPOV() == 90){
        if (pov90 == 0){
          if (speedChannel < 4){
            speedChannel++;
          }
          pov90 = 1;
        }
      }
      else{
        pov90 = 0;
      }
  
      if (con2.getPOV() == 270){
        if (pov270 == 0){
          if (speedChannel > 0){
            shootUpSpeedChannel--;            
          }
          pov270 = 1;
        }
      }
      else{
        pov270 = 0;
      }

//////////////////
// robotDrive.driveCartesian(0.2,0 ,0);

if (Math.abs(con2.getLeftX()) > 0.1 || Math.abs(con2.getLeftY()) > 0.1 || Math.abs(con2.getRightX()) > 0.1)
    robotDrive.driveCartesian(-con2.getLeftY() * speed[speedChannel], con2.getLeftX() * speed[speedChannel], con2.getRightX() * speed[speedChannel]);
else
    robotDrive.driveCartesian(0, 0, 0);
      
            

    //--------------------------------------climb↑-------------------------------------
    //---------------------------------------ball↓-------------------------------------
      // //////////inTakeOut//////////
      // if (con1.getXButton()) {
      //   inTakeOut.set(0.5);
      // }
      // else if(con1.getYButton()){
      //   inTakeOut.set(-0.5);
      // }
      // else{
      //   inTakeOut.set(0);
      // }
        
      // /////stop_trasportdown & inTakeOn//////
      // if (con1.getRightTriggerAxis() > 0.1) {
      //   transportDown.set(0);
      //   inTakeOn.set(0);
      // }

      // ///////trasportdown & inTakeOn//////////
      // if (con1.getAButtonPressed()) {
      //   if (con1_AButtonPressed) {
      //     transportDown.set(0.4);
      //     // if(Voltage>12.8)
      //     //   inTakeOn.set(0.5);
      //     // else if(Voltage>12)
      //     //   inTakeOn.set(0.6);
      //     // else
      //     //   inTakeOn.set(0.7);
      //     inTakeOn.set(0.8);

      //   } 
      //   else {
      //     transportDown.set(-0.4);
      //     // if(Voltage>12.8)
      //     //   inTakeOn.set(-0.5);
      //     // else if(Voltage>12)
      //     //   inTakeOn.set(-0.6);
      //     // else
      //     //   inTakeOn.set(-0.7);
      //     inTakeOn.set(-0.8);
      //   }
      //   con1_AButtonPressed = !con1_AButtonPressed;
      // }

      // //////////////green_wheel////////////
      // if (con1.getBButtonPressed()) {
      //   if (con1_BButtonPressed)
      //     transportUp.set(0.5);
      //   else
      //     transportUp.set(0);
      //   con1_BButtonPressed = !con1_BButtonPressed;
      // }

      // /////////////shoot/////////////
      // if (con1.getPOV() == 0){
      //   if (pov0 == 0){
      //     if (shootUpSpeedChannel != 3){
      //       shootUpSpeedChannel+=1;
      //     }
      //     pov0 = 1;
      //   }
      // }
      // else{
      //   pov0 = 0;
      // }

      // if (con1.getPOV() == 180){
      //   if (pov180 == 0){
      //     if (shootUpSpeedChannel != 0){
      //       shootUpSpeedChannel-=1;            
      //     }
      //     pov180 = 1;
      //   }
      // }
      // else{
      //   pov180 = 0;
      // }
      
      // if (con1.getPOV() == 90){
      //   if (pov90 == 0){
      //     if (shootDownSpeedChannel != 3){
      //       shootDownSpeedChannel+=1;
      //     }
      //     pov90 = 1;
      //   }
      // }
      // else{
      //   pov90 = 0;
      // }
      
      // if (con1.getPOV() == 270){
      //   if (pov270 == 0){
      //     if (shootDownSpeedChannel != 0){
      //       shootDownSpeedChannel-=1;
      //     }
      //     pov270 = 1;
      //   }
      // }
      // else{
      //   pov270 = 0;
      // }
      // // System.out.println(shootUpSpeedChannel);
      // // System.out.println(shootDownSpeedChannel);

      // if (con1.getLeftTriggerAxis() > 0.1) {
      //   PID_shootUp.setReference(shootUpSpeed[shootUpSpeedChannel], ControlType.kVelocity);
      //   PID_shootDown.setReference(shootDownSpeed[shootDownSpeedChannel], ControlType.kVelocity);
      // } 
      // else {
      //   PID_shootUp.setReference(0, ControlType.kVelocity);
      //   PID_shootDown.setReference(0, ControlType.kVelocity);
      // }


      // //////////////////////tankdrive//////////////////////
    
      // if (con1.getRightBumperPressed() && speedChannel < 4) speedChannel++;
      // if (con1.getLeftBumperPressed() && speedChannel > 0) speedChannel--;
    
        

      // if (Math.abs(con2.getLeftY()) > 0.1){
      //   leftdrive.set(-con2.getLeftY() * speed[speedChannel]);
      //   // robotDrive.driveCartesian(-con1.getLeftX() * speed[speedChannel], con1.getLeftY() * speed[speedChannel], con1.getRightX() * speed[speedChannel]);
      // }
      // else{
      //   leftdrive.set(0);
      //   // robotDrive.driveCartesian(0, 0, 0);
      // }
      // if (Math.abs(con2.getRightY()) > 0.1){
      //   rightdrive.set(con2.getRightY() * speed[speedChannel]);
      // }
      // else{
      //   rightdrive.set(0);
      // }

  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {
    // victorfrontLeft = new WPI_VictorSPX(10);
    // victorbackLeft = new WPI_VictorSPX(3);
    // victorfrontRight = new WPI_VictorSPX(0);
    // victorbackRight = new WPI_VictorSPX(6);
    // if(con1.getAButton())
    // victorfrontLeft.set(1);  //0
    // else
    //   victorfrontLeft.set(0);
    

    // if(con1.getBButton())
    // victorbackLeft.set(1);   //3 
    // else
    // victorbackLeft.set(0);

    // if(con1.getXButton())
    // victorfrontRight.set(1);//10 
    // else
    // victorfrontRight.set(0);

    // if(con1.getYButton())
    // victorbackRight.set(1); //6
    // else
    //   victorbackRight.set(0);
    }
}
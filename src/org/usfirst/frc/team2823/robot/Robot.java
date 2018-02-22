/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2823.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	Joystick driveStick;
	Joystick operatorStick;
	
	final int xButton = 1;
	final int aButton = 2;
	final int bButton = 3;
	final int yButton = 4;
	final int leftBumper = 5;
	final int rightBumper = 6;
	final int leftTrigger = 7;
	final int rightTrigger = 8;
	final int startButton = 10;
	
	Button intakeOpenButton;
	Button toggleIntakeDftButton; 
	Button intakeOutButton;
	Button unClampButton;
	Button gearHighButton;
	Button gearLowButton;
	
	Button testButton;
	Button resetButton;

	TalonSRX leftMotor1;
	TalonSRX leftMotor2;
	TalonSRX rightMotor3;
	TalonSRX rightMotor4;
	
	VictorSPX leftElbow;
	VictorSPX rightElbow;
	VictorSPX leftBelt;
	VictorSPX rightBelt;
	
	TalonSRX fourbarMotor;
	VictorSPX elevatorMotor;

	DoubleSolenoid clamper;
	DoubleSolenoid driveSolenoid;
	Compressor compressor;
	
	Encoder lDriveEncoder;
	Encoder rDriveEncoder;
	Encoder lElbowEncoder;
	Encoder rElbowEncoder;
	Encoder fourbarEncoder;
	Encoder elevatorEncoder;
	
	ADXRS450_Gyro gyro;
	
	DriveInchesPIDSource leftInches;
	DriveInchesPIDSource rightInches;
	
	
	SnazzyPIDController leftPIDControl;
	SnazzyPIDController rightPIDControl;
	
	SnazzyPIDController leftIntakeControl;
	SnazzyPIDController rightIntakeControl;
	
	final double lStow = 135.0;
	final double rStow = 115.0;
	final double open = 278.0;
	final double grab = 230.0;
	final double start = 0.0;
	final double clear = 248.0;
	double rIntakeSetpoint = 0.0;
	double lIntakeSetpoint = 0.0;
	double stowDelay = 0.0;
	boolean goinDown = false;
	boolean goinUp = false;
	final double stowFudge = 10;
	final double elbowFudge = 3;
	final double autoStowThreshold = 500;
	String intakeIndicator = "Start";
	
	SnazzyPIDController fourbarPIDControl;
	//SnazzyMotionPlanner fourbarMotionControl;
	FourbarOutput fourbarOutput;
	
	double fourbarSetpoint = 0.0;
	final double downStep = 1000.0;
	final double upStep = 1000.0;
	final double fourbarLastBit = downStep*8;
	final double fourbarUpperLimit = 80000;
	final double fourbarLowerLimit = 0;
	final double upperSafeZoneLimit = 26000;
	final double lowerSafeZoneLimit = 500;
	
	SnazzyMotionPlanner leftControl;
	SnazzyMotionPlanner rightControl;
	
	LeftDrivePIDOutput lDriveOutput;
	RightDrivePIDOutput rDriveOutput;
	
	final double lowGearKA = 0.00246*1.15;
	final double lowGearKV = 0.0108;
	final double highGearKA = 0.00380;
	final double highGearKV = 0.00623;
	final double highGearP = 0.2;
	final double highGearI = 0.00003;
	final double highGearD = 3.0;
	final double lowGearP = 0.3;
	final double lowGearI = 0.005;
	final double lowGearD = 1.0;
	
	LeftIntakeOutput lIntakeOutput;
	RightIntakeOutput rIntakeOutput;
	
	TrajectoryPlanner rightSwitchAutoTraj;
	TrajectoryPlanner rightSwitchBackTraj;
	TrajectoryPlanner leftSwitchAutoTraj;
	TrajectoryPlanner leftSwitchBackTraj;
	
	TrajectoryPlanner switchGrabCubeTraj;
	
	TrajectoryPlanner scaleStartAutoTraj;
	TrajectoryPlanner leftScaleEndTraj;
	TrajectoryPlanner rightScaleFirstTurnTraj;
	TrajectoryPlanner rightScaleMidTraj;
	TrajectoryPlanner rightScaleEndTraj;

	
	TrajectoryPlanner driveForwardTraj;
	
	double[][] rightSwitchAutoPlan = {{0,0,0},{100.5, -54.25 - 8, 0}};
	double[][] rightSwitchBackPlan = {{20, 5.25, 0}, {100.5, -54.25 - 8, 0}};
	
	double[][] leftSwitchAutoPlan = {{0,0,0},{100.5, 64.75 + 12, 0}};
	double[][] leftSwitchBackPlan = {{20, 5.25 + 5, 0}, {100.5, 64.75 + 12, 0}};
	
	double[][] switchGrabCubePlan = {{0,0,0}, {38.5 + 6, 0, 0}};
	
	double[][] scaleStartAutoPlan = {{0,0,0},{160, 0, 0}};
	double[][] leftScaleEndPlan = {{160, 0,0},{280 + 12, -10 - 12, -45}};
	double[][] rightScaleFirstTurnPlan = {{160, 0,0},{220, -60,-90}};
	double[][] rightScaleMidPlan = {{220, -60,-90},{220, -171, -90}};
	double[][] rightScaleEndPlan = {{220, -120, -90}, {260 + 12, -25 + 12, 45}};	
	
	double[][] driveForwardPlan = {{0,0,0}, {90, 0, 0}};

	boolean calibrate = false;
	boolean pidTune = false;
	
	boolean clamped = false;
	
	SendableChooser<Autonomous> autonomousChooser;
	
	final static double ENC_TO_INCH = Math.PI * 6.0 * (24.0/60.0) * (1.0/3.0) * (1.0/256.0);
	final static double INCH_TO_ENC = 1/ENC_TO_INCH;
	
	/*  1 spin of the wheel corresponds to 60/24 spins of the output axle (that's our 3rd stage gearing).
	 *  The output axle is connected to a 36 t 'encoder gear' which spins another 12t 'encoder gear', so the encoder axles spins at 3 times that rate. 
	 *  Then we have 256 ticks per revolution.  So I think that's a total of 60 / 24 * 3 * 256 = 1920  ticks per revolution
	 */
	
	DoubleSolenoid.Value highGear = DoubleSolenoid.Value.kReverse;
	DoubleSolenoid.Value lowGear = DoubleSolenoid.Value.kForward;
	
	DoubleSolenoid.Value clampIt = DoubleSolenoid.Value.kForward;
	DoubleSolenoid.Value unClampIt = DoubleSolenoid.Value.kReverse;
	
	double transmissionUpper = 75.0;
	double transmissionLower = 55.0;
	
	double velocity = 0.0;
	double rCurrentDist = 0.0;
	double lCurrentDist = 0.0;
	double currentTime = 0.0;
	int velocitySample = 20;
	ArrayList <Double> lLastDistances;
	ArrayList <Double> rLastDistances;
	ArrayList <Double> lastTimes;
	boolean manual = false;
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		autonomousChooser = new SendableChooser<Autonomous>();
		autonomousChooser.addDefault("Empty: Do Nothing", new EmptyAutonomous(this));
		autonomousChooser.addObject("Drive Forward", new DriveForwardAuto(this));
		autonomousChooser.addObject("Switch Auto", new SwitchAuto(this));
		autonomousChooser.addObject("Scale Auto", new ScaleAuto(this));
		autonomousChooser.addObject("Do a Spin", new SpinnyAuto(this));
		SmartDashboard.putData("Autonomous Mode", autonomousChooser);
		
		/** UsbCamera c = CameraServer.getInstance().startAutomaticCapture();
	        c.setResolution(320, 180);
	        c.setFPS(29);
	        **/
		driveStick = new Joystick(0);
		operatorStick = new Joystick(1);
		
		lLastDistances = new ArrayList<Double>(velocitySample);
		rLastDistances = new ArrayList<Double>(velocitySample);
		lastTimes = new ArrayList<Double>(velocitySample);

		intakeOpenButton = new Button();
		toggleIntakeDftButton = new Button();
		intakeOutButton = new Button();
		unClampButton = new Button();
		gearHighButton = new Button();
		gearLowButton = new Button();
		
		testButton = new Button();
		resetButton = new Button();
		
		leftMotor1 = new TalonSRX(1);
		leftMotor2 = new TalonSRX(2);
		rightMotor3 = new TalonSRX(3);
		rightMotor4 = new TalonSRX(4);
		
		leftMotor1.configOpenloopRamp(0, 0);
		leftMotor2.configOpenloopRamp(0, 0);
		rightMotor3.configOpenloopRamp(0, 0);
		rightMotor4.configOpenloopRamp(0, 0);
		
		leftElbow = new VictorSPX(11);
		rightElbow = new VictorSPX(12);
		leftBelt = new VictorSPX(21);
		rightBelt = new VictorSPX(22);
		
		fourbarMotor = new TalonSRX(31);
		fourbarOutput = new FourbarOutput(this);
		//elevatorMotor = new VictorSPX(41);
			
		lDriveEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		lDriveEncoder.setDistancePerPulse(1);
		rDriveEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
		rDriveEncoder.setDistancePerPulse(1);
		
		lElbowEncoder = new Encoder(22, 23, false, Encoder.EncodingType.k4X);
		rElbowEncoder = new Encoder(16, 17, false, Encoder.EncodingType.k4X);
		
		leftInches = new DriveInchesPIDSource(lDriveEncoder);
		rightInches = new DriveInchesPIDSource(rDriveEncoder);
		
		fourbarEncoder = new Encoder(8,9, true, Encoder.EncodingType.k4X);
		elevatorEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k4X);

		clamper = new DoubleSolenoid(0,1);
		driveSolenoid = new DoubleSolenoid(2, 3);
		compressor = new Compressor(0);

		driveSolenoid.set(highGear);
		clamper.set(clampIt);

		lDriveEncoder.reset();
		rDriveEncoder.reset();
		lElbowEncoder.reset();
		rElbowEncoder.reset();
		fourbarEncoder.reset();
		elevatorEncoder.reset();
		
		//gyro = new ADXRS450_Gyro();
		//gyro.reset();
	
		lDriveOutput = new LeftDrivePIDOutput(this);
		rDriveOutput = new RightDrivePIDOutput(this);
		
		lIntakeOutput = new LeftIntakeOutput(this);
		rIntakeOutput = new RightIntakeOutput(this);
		
		rightSwitchAutoTraj = new TrajectoryPlanner(rightSwitchAutoPlan, 100*0.5, 300, 300); //the integers are what the chassis is capable of, then we limit it with the decimals
		rightSwitchAutoTraj.generate();
		
		leftSwitchAutoTraj = new TrajectoryPlanner(leftSwitchAutoPlan, 100*0.5, 300, 300);
		leftSwitchAutoTraj.generate();
		
		leftSwitchBackTraj = new TrajectoryPlanner(leftSwitchBackPlan, 100*0.5, 300, 300);
		leftSwitchBackTraj.generate();
		
		rightSwitchBackTraj = new TrajectoryPlanner(rightSwitchBackPlan, 100*0.5, 300, 300);
		rightSwitchBackTraj.generate();
		
		switchGrabCubeTraj = new TrajectoryPlanner(switchGrabCubePlan, 100*0.5, 300, 300);
		switchGrabCubeTraj.generate();
		
		scaleStartAutoTraj = new TrajectoryPlanner(scaleStartAutoPlan, 140, 600, 600);
		scaleStartAutoTraj.generate();
		
		leftScaleEndTraj = new TrajectoryPlanner(leftScaleEndPlan, 100 *0.5, 300, 300);
		leftScaleEndTraj.generate();
		
		rightScaleFirstTurnTraj = new TrajectoryPlanner(rightScaleFirstTurnPlan, 100 *0.5, 300, 300);
		rightScaleFirstTurnTraj.generate();
		
		rightScaleMidTraj = new TrajectoryPlanner(rightScaleMidPlan, 140, 600, 600);
		rightScaleMidTraj.generate();
		
		rightScaleEndTraj = new TrajectoryPlanner(rightScaleEndPlan, 100*0.5, 300, 300);
		rightScaleEndTraj.generate();
		
		driveForwardTraj = new TrajectoryPlanner(driveForwardPlan, 100*0.5, 300, 300);
		driveForwardTraj.generate();
		
		leftControl = new SnazzyMotionPlanner(lowGearP, lowGearI, lowGearD, 0, lowGearKA, lowGearKV,  leftInches, lDriveOutput, 0.005, "Left.csv");
		rightControl= new SnazzyMotionPlanner(lowGearP, lowGearI, lowGearD, 0, lowGearKA, lowGearKV,  rightInches, rDriveOutput, 0.005,"Right.csv");
		
		leftPIDControl = new SnazzyPIDController(0.04, 0.001, 0.8, 0, leftInches, lDriveOutput, 0.005, "Left.csv");
		rightPIDControl= new SnazzyPIDController(0.04, 0.001, 0.8, 0, rightInches, rDriveOutput, 0.005,"Right.csv");
		
		fourbarPIDControl = new SnazzyPIDController(0.0005, 0.0, 0.0, 0.0, fourbarEncoder, fourbarOutput, 0.005, "FourbarPID.csv");
		fourbarPIDControl.setOutputRange(-0.8, 1.0);
		
		fourbarSetpoint = 0;
		//fourbarMotionControl = new SnazzyMotionPlanner(0.0005, 0.0, 0.0, 0.0, 0.00000310,0.00000387, fourbarEncoder, fourbarOutput, 0.005, "FourbarMotion.csv");
		
		leftIntakeControl = new SnazzyPIDController(0.05, 0, 0, 0, lElbowEncoder, lIntakeOutput, 0.005, "leftIntake.csv");
		rightIntakeControl = new SnazzyPIDController(0.05, 0, 0, 0, rElbowEncoder, rIntakeOutput, 0.005, "rightIntake.csv");
		
		rIntakeSetpoint = start;
		lIntakeSetpoint = start;
		
		SmartDashboard.putNumber("P", 0.01);
		SmartDashboard.putNumber("I", 0.005);
		SmartDashboard.putNumber("D", 1.0);
		SmartDashboard.putNumber("Setpoint", 0);
		SmartDashboard.putNumber("L Encoder", 0);
		SmartDashboard.putNumber("R Encoder", 0);
		SmartDashboard.putNumber("L Elbow", 0);
		SmartDashboard.putNumber("R Elbow", 0);
		SmartDashboard.putString("Driving Gear", "Low");
		SmartDashboard.putString("Intake Setpoint", "Start");
		SmartDashboard.putNumber("Fourbar", 0);
		SmartDashboard.putString("Clamped?", "Not Clamped");
		
	}

	@Override
	public void autonomousInit() {
		
		lDriveEncoder.reset();
		rDriveEncoder.reset();
		lElbowEncoder.reset();
		rElbowEncoder.reset();
		fourbarEncoder.reset();
		elevatorEncoder.reset();
		
		leftControl.reset();
		rightControl.reset();
		
		lIntakeSetpoint = start;
		leftIntakeControl.setSetpoint(start);
		rIntakeSetpoint = start;
		rightIntakeControl.setSetpoint(start);
		
		enableMechanismPIDs();
		
		driveSolenoid.set(lowGear);
		
		
		((Autonomous) autonomousChooser.getSelected()).init();
		
		
	}

	@Override
	public void autonomousPeriodic() {
		((Autonomous) autonomousChooser.getSelected()).periodic();
		
		SmartDashboard.putNumber("L Encoder", lDriveEncoder.get());
		SmartDashboard.putNumber("R Encoder", rDriveEncoder.get());
	}
	
	@Override
	public void teleopInit() {
		leftControl.disable();
		rightControl.disable();
		enableMechanismPIDs();
		
	}

	@Override
	public void teleopPeriodic() {
		
		calibrate = false;
		pidTune = false;
		
		if(!calibrate && !pidTune) {			
			gearLowButton.update(driveStick.getRawButton(leftTrigger) || operatorStick.getRawButton(leftTrigger));
			gearHighButton.update(driveStick.getRawButton(leftBumper) || operatorStick.getRawButton(leftBumper));
			intakeOpenButton.update(operatorStick.getRawButton(rightBumper));
			toggleIntakeDftButton.update(operatorStick.getRawButton(yButton));
			intakeOutButton.update(operatorStick.getRawButton(rightTrigger));
			unClampButton.update(operatorStick.getRawButton(bButton));
			testButton.update(driveStick.getRawButton(1));
			resetButton.update(driveStick.getRawButton(startButton) || operatorStick.getRawButton(startButton));
		
			
			currentTime = Timer.getFPGATimestamp();
			rCurrentDist = rightInches.pidGet();
			lCurrentDist = leftInches.pidGet();
			lLastDistances.add(lCurrentDist);
			rLastDistances.add(rCurrentDist);
			lastTimes.add(currentTime);
			
			if(resetButton.changed()) {
				lElbowEncoder.reset();
				rElbowEncoder.reset();
				fourbarEncoder.reset();
				elevatorEncoder.reset();
			}
			
			if(!unClampButton.on()) {
				clamper.set(clampIt);
				clamped = true;
				if(fourbarEncoder.get()<= autoStowThreshold && unClampButton.changed()) {
					toggleIntakeDftButton.reset();
				}
	
				
			}else if(unClampButton.on()) {
				clamper.set(unClampIt);
				clamped = false;
			}
			
			//Automatic Transmission
			/*if(lLastDistances.size()==velocitySample && rLastDistances.size() == velocitySample ) {
				velocity  = ((lCurrentDist-lLastDistances.get(0))/(currentTime-lastTimes.get(0))+(rCurrentDist-rLastDistances.get(0))/(currentTime-lastTimes.get(0)))/2;
				if(Math.abs(velocity) >= transmissionUpper && driveSolenoid.get() == lowGear && !manual) {
					driveSolenoid.set(highGear);
					SmartDashboard.putString("Driving Gear", "High");
				} 
				
				if(Math.abs(velocity) <= transmissionLower && driveSolenoid.get() == highGear && !manual) {
					driveSolenoid.set(lowGear);
					SmartDashboard.putString("Driving Gear", "Low");
				}
				
				lLastDistances.remove(0);
				rLastDistances.remove(0);
				lastTimes.remove(0);
			}*/
			
			//fourbar stuff
			if((driveStick.getPOV() == 0 || operatorStick.getPOV() == 0) && fourbarSetpoint <fourbarUpperLimit && fourbarIsSafe(fourbarSetpoint+upStep)) {
				fourbarSetpoint += upStep;
			}
			if((driveStick.getPOV()== 0 || operatorStick.getPOV() == 0) && fourbarSetpoint <fourbarUpperLimit && !fourbarIsSafe(fourbarSetpoint+upStep)) {
				goinUp = true;
			}else {
				goinUp = false;
			}
			if((driveStick.getPOV()== 180 || operatorStick.getPOV()== 180) && fourbarSetpoint > fourbarLowerLimit && fourbarIsSafe(fourbarSetpoint-downStep)) {
				if(fourbarSetpoint < fourbarLastBit) {
					fourbarSetpoint -= downStep/2;
				} else {
				fourbarSetpoint -= downStep;
				}
			} 
			if((driveStick.getPOV()== 180 || operatorStick.getPOV()== 180) && fourbarSetpoint >fourbarLowerLimit && !fourbarIsSafe(fourbarSetpoint-downStep)) {
				goinDown = true;
			}else {
				goinDown = false;
			}
			
			SmartDashboard.putNumber("Setpoint", fourbarSetpoint);
			fourbarPIDControl.setSetpoint(fourbarSetpoint);
			
			//intake stuff
			if((lElbowEncoder.get()<=(-lStow + stowFudge) && rElbowEncoder.get()>=(rStow-stowFudge) && fourbarEncoder.get()<= lowerSafeZoneLimit) || fourbarEncoder.get()>= upperSafeZoneLimit) {
				if(toggleIntakeDftButton.on()) {
					stowDelay = 0.0;
					if(intakeOpenButton.held() && !intakeOutButton.held()) {
						rIntakeSetpoint = open;
						lIntakeSetpoint = open;
					} else if(!intakeOpenButton.held() && intakeOutButton.held()) {
						rIntakeSetpoint = grab;
						lIntakeSetpoint = grab;
						
					}else {
						rIntakeSetpoint = grab;
						lIntakeSetpoint = grab;
					}
					
				} else if(!toggleIntakeDftButton.on()) {
					if(intakeOpenButton.held() && !intakeOutButton.held()) {
						rIntakeSetpoint = open;
						lIntakeSetpoint = open;
						stowDelay = 0.0;
						
					} else if(!intakeOpenButton.held() && intakeOutButton.held()) {
						rIntakeSetpoint = grab;
						lIntakeSetpoint = grab;
						stowDelay = 0.0;
						
						
					}else {
						
						stowDelay += 1;
						if(Math.abs(lElbowEncoder.get())<= lStow && rElbowEncoder.get() <= rStow) {
							lIntakeSetpoint = lStow;
							if(stowDelay >= 20) {
								rIntakeSetpoint = rStow;
							}
						} else {
							rIntakeSetpoint = rStow;
							if(stowDelay >= 20) {
								lIntakeSetpoint = lStow;
							}
						}
						
					}
				}
			} 
			
			if(goinDown) {
				lIntakeSetpoint = clear;
				rIntakeSetpoint = clear;
				unClampButton.reset();
				stowDelay = 0.0;
			}
			if(goinUp) {
				lIntakeSetpoint = clear;
				rIntakeSetpoint = clear;
				stowDelay = 0.0;
			}
			
			if(rIntakeSetpoint >= grab && !intakeOutButton.held() && !clamped) {
				leftBelt.set(ControlMode.PercentOutput, -1.0);
				rightBelt.set(ControlMode.PercentOutput, 1.0);
			} else if(intakeOutButton.held() && !clamped){
				leftBelt.set(ControlMode.PercentOutput, 1.0);
				rightBelt.set(ControlMode.PercentOutput, -1.0);
				
			}else {
				leftBelt.set(ControlMode.PercentOutput, 0.0);
				rightBelt.set(ControlMode.PercentOutput, 0.0);
			}
			
			leftIntakeControl.setSetpoint(-lIntakeSetpoint);
			
			rightIntakeControl.setSetpoint(rIntakeSetpoint);
			
	
			if (gearLowButton.held()) {
				driveSolenoid.set(lowGear);
				SmartDashboard.putString("Driving Gear", "Low");
				
			}else {
				driveSolenoid.set(highGear);
				SmartDashboard.putString("Driving Gear", "High");
			}
			
			SmartDashboard.putNumber("L Encoder", leftInches.pidGet());
			SmartDashboard.putNumber("R Encoder", rightInches.pidGet());
			SmartDashboard.putNumber("L Elbow", lElbowEncoder.get());
			SmartDashboard.putNumber("R Elbow", rElbowEncoder.get());
			SmartDashboard.putNumber("Fourbar", fourbarEncoder.get());
			
			switch((int)rIntakeSetpoint) {
				case (int)start:
					intakeIndicator = "Start";
					break;
				case (int)grab:
					intakeIndicator = "Grab";
					break;
				case (int)open:
					intakeIndicator = "Open";
					break;
				case (int) clear:
					intakeIndicator = "Clear";
					break;
				case (int)rStow:
					intakeIndicator = "Stow";
					break;
					
			}
			
			if(clamped) {
				SmartDashboard.putString("Clamped?", "It's Clamped");
			} else {
				SmartDashboard.putString("Clamped?", "Not Clamped");

			}
					
					
			SmartDashboard.putString("Intake Setpoint", intakeIndicator);

			
			leftMotor1.set(ControlMode.PercentOutput, Math.pow(driveStick.getRawAxis(1), 3));
			leftMotor2.set(ControlMode.PercentOutput, Math.pow(driveStick.getRawAxis(1), 3) );
			rightMotor3.set(ControlMode.PercentOutput, -Math.pow(driveStick.getRawAxis(3), 3));
			rightMotor4.set(ControlMode.PercentOutput, -Math.pow(driveStick.getRawAxis(3), 3));
			
			//elevatorMotor.set(ControlMode.PercentOutput, operatorStick.getRawAxis(3)*.4);
			
		
		} else if(calibrate && !pidTune) {
			calibrateNow();
		}else if(!calibrate && pidTune) {
			pidTune();
		}
		
	}


	@Override
	public void testInit() {
		leftControl.disable();
		rightControl.disable();
	}

	@Override
	public void testPeriodic() {
		
	}
	public void disabledInit() {
		
	}
	
	public void calibrateNow() {
		testButton.update(driveStick.getRawButton(1));
		driveSolenoid.set(lowGear);
		
		if(testButton.on()){
			if(testButton.changed()) {
				lDriveEncoder.reset();
				rDriveEncoder.reset();
				
				leftControl.startCalibration();
				rightControl.startCalibration();
				
				leftControl.enable();
				rightControl.enable();
				
				//fourbarMotionControl.configureGoal(60000, 130000*0.5, 3000000*0.5, false);
				//fourbarMotionControl.enable();
				
			}
			
		}else if (testButton.changed()&& !testButton.on()){
			leftControl.disable();
			rightControl.disable();		
			//fourbarMotionControl.disable();
			
		}
		SmartDashboard.putNumber("L Encoder", leftInches.pidGet());
		SmartDashboard.putNumber("R Encoder", rightInches.pidGet());
	}
	
	public void pidTune() {
		testButton.update(driveStick.getRawButton(1));
		
		driveSolenoid.set(lowGear);
		//driveSolenoid.set(highGear);
		
		leftControl.setkAkV(lowGearKA, lowGearKV);
		rightControl.setkAkV(lowGearKA, lowGearKV);
		
		//leftControl.setkAkV(highGearKA, highGearKV);
		//rightControl.setkAkV(highGearKA, highGearKV);
		
		leftControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		rightControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//leftPIDControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//rightPIDControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//fourbarPIDControl.setPID(SmartDashboard.getNumber("P", 0),SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//leftIntakeControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//rightIntakeControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
	
		if(testButton.on()){
				if(testButton.changed()) {
					lDriveEncoder.reset();
					rDriveEncoder.reset();
					
					//fourbarEncoder.reset();
					
					lElbowEncoder.reset();
					
					//leftPIDControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					//leftPIDControl.enable();
					
					//leftControl.configureTrajectory(driveForwardTraj.getLeftTrajectory(), false);
					//rightControl.configureTrajectory(driveForwardTraj.getRightTrajectory(), false);
					
					leftControl.configureTrajectory(rightSwitchAutoTraj.getLeftTrajectory(), false);
					rightControl.configureTrajectory(rightSwitchAutoTraj.getRightTrajectory(), false);

					
					//leftIntakeControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					//leftIntakeControl.enable();
					//rightIntakeControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					//rightIntakeControl.enable();
					
					//fourbarPIDControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					//fourbarPIDControl.enable();
					
					//rightPIDControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					//rightPIDControl.enable();
					leftControl.enable();
					rightControl.enable();
	
				}
				
			}else if (testButton.changed()&& !testButton.on()){
				//leftPIDControl.disable();
				//rightPIDControl.disable();
				leftControl.disable();
				rightControl.disable();
				//fourbarPIDControl.disable();
				//leftIntakeControl.disable();
				//rightIntakeControl.disable();
				
			}
		SmartDashboard.putNumber("L Encoder", leftInches.pidGet());
		SmartDashboard.putNumber("R Encoder", rightInches.pidGet());
		SmartDashboard.putNumber("Fourbar", fourbarEncoder.get());
		SmartDashboard.putNumber("L Elbow", lElbowEncoder.get());
		SmartDashboard.putNumber("R Elbow", rElbowEncoder.get());
	}
	
	public boolean fourbarIsSafe(double futureSetpoint) {
		// 18000 safe for arm
		if(futureSetpoint >= upperSafeZoneLimit) {
			return true;
		}
		if(Math.abs(lElbowEncoder.get()-start)<= elbowFudge && Math.abs(rElbowEncoder.get()-start)<=  elbowFudge) {
			return true;
		}
		if(Math.abs(Math.abs(lElbowEncoder.get())-clear)<= elbowFudge && Math.abs(Math.abs(rElbowEncoder.get())-clear)<= elbowFudge) {
			return true;
		}
		return false;
	}
	
	public void enableMechanismPIDs() {
		fourbarPIDControl.enable();
		leftIntakeControl.enable();
		rightIntakeControl.enable();
	}
	
	public void disableMechanismPIDs() {
		fourbarPIDControl.disable();
		leftIntakeControl.disable();
		rightIntakeControl.disable();
	}
	
	public void configureToGear(DoubleSolenoid.Value gear) {
		if(gear == lowGear) {
			driveSolenoid.set(lowGear);
			leftControl.setkAkV(lowGearKA, lowGearKV);
			leftControl.setPID(lowGearP, lowGearI, lowGearD);
			rightControl.setPID(lowGearP, lowGearI, lowGearD);
		} else {
			driveSolenoid.set(highGear);
			leftControl.setkAkV(highGearKA, highGearKV);
			leftControl.setPID(highGearP, highGearI, highGearD);
			rightControl.setPID(highGearP, highGearI, highGearD);
		}
	}
}

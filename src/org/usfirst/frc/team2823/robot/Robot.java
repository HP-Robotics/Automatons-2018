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

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
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
	final int backButton = 9;
	final int startButton = 10;
	
	Button intakeOpenButton;
	Button toggleIntakeDftButton; 
	Button intakeOutButton;
	Button unClampButton;
	Button gearHighButton;
	Button gearLowButton;
	Button toggleShiftMode;
	Button elevatorUpButton;
	Button elevatorDownButton;
	
	Button testButton;
	Button elbowResetButton;
	Button nannyModeOffButton; 

	TalonSRX leftMotor1;
	TalonSRX leftMotor2;
	TalonSRX rightMotor3;
	TalonSRX rightMotor4;
	
	VictorSPX leftElbow;
	VictorSPX rightElbow;
	VictorSPX leftBelt;
	VictorSPX rightBelt;
	
	//TalonSRX fourbarMotor;
	VictorSPX fourbarMotor;//OWEN CHANGED FROM TALONSRX TO VICTORSPX 3/17/18 FOR SHADOW ROBOT  TODO
	VictorSP elevatorMotor;// TODO HACKED TO BE VictorSP not SPX for Shadow

	DoubleSolenoid clamper;
	DoubleSolenoid driveSolenoid;
	Compressor compressor;
	
	Encoder lDriveEncoder;
	Encoder rDriveEncoder;
	Encoder lElbowEncoder;
	Encoder rElbowEncoder;
	Encoder fourbarEncoder;
	Encoder elevatorEncoder;
	
	Potentiometer rPot;
	Potentiometer lPot;
	AnalogInput ai1 = new AnalogInput(1);
	AnalogInput ai2 = new AnalogInput(2);
	
	ADXRS450_Gyro gyro;
	
	DriveInchesPIDSource leftInches;
	DriveInchesPIDSource rightInches;
	
	
	SnazzyPIDController leftPIDControl;
	SnazzyPIDController rightPIDControl;
	
	SnazzyPIDController leftIntakeControl;
	SnazzyPIDController rightIntakeControl;
	
	final double lStow = 109.0;
	final double rStow = 240.0;
	final double lOpen = 205; //220  OWEN TODO LIMIT FOR SHADOW
	final double rOpen = 127; //105  OWEN TODO LIMIT FOR SHADOW
	final double lGrab = 184.0;
	final double rGrab = 144.0;
	final double lStart = 2.0;
	final double rStart = 343.0;
	final double lClear = 199.0;
	final double rClear = 128.0;
	double rIntakeSetpoint = 0.0;
	double lIntakeSetpoint = 0.0;
	double stowDelay = 0.0;
	boolean goinDown = false;
	boolean goinUp = false;
	final double stowFudge = 10;
	final double elbowFudge = 3;
	final double autoStowThreshold = 500;
	String intakeIndicator = "Start";
	double rResetOffset = 0.0;
	double lResetOffset = 0.0;
	double rPotStart = 0.0;
	double lPotStart = 0.0;
	double lElbowLimit = 352.0;
	double rElbowLimit = 343.0;
	
	final double rBeltSpeed = 1.0;
	final double lBeltSpeed = -1.0;
	
	SnazzyPIDController fourbarPIDControl;
	//SnazzyMotionPlanner fourbarMotionControl;
	FourbarOutput fourbarOutput;
	
	double fourbarSetpoint = 0.0;
	final double downStep = 1000.0;
	final double upStep = 1000.0;
	final double fourbarLastBit = downStep*8;
	final double fourbarUpperLimit = 90000;
	final double fourbarLowerLimit = 0;
	final double upperSafeZoneLimit = 30000;
	final double lowerSafeZoneLimit = 1500;
	
	SnazzyPIDController elevatorPIDControl;
	SnazzyMotionPlanner elevatorMotionControl;
	ElevatorOutput elevatorOutput;
	
	SnazzyMotionPlanner leftControl;
	SnazzyMotionPlanner rightControl;
	
	LeftDrivePIDOutput lDriveOutput;
	RightDrivePIDOutput rDriveOutput;
	
	final double lowGearKA = 0.00246*1.15;
	final double lowGearKV = 0.0108;
	final double highGearKA = 0.00380;
	final double highGearKV = 0.00623;
	final double highGearP = 0.7;
	final double highGearI = 0.002;
	final double highGearD = 0.0;
	final double lowGearP = 0.3;
	final double lowGearI = 0.005;
	final double lowGearD = 1.0;
	
	double maxPow = 1.0;
	
	boolean elevatorUp = false;
	
	LeftIntakeOutput lIntakeOutput;
	RightIntakeOutput rIntakeOutput;
	
	TrajectoryPlanner rightSwitchAutoTraj;
	TrajectoryPlanner rightSwitchBackTraj;
	TrajectoryPlanner leftSwitchAutoTraj;
	TrajectoryPlanner leftSwitchBackTraj;
	
	TrajectoryPlanner switchGrabCubeTraj;
	
	TrajectoryPlanner leftScaleStartTraj;
	TrajectoryPlanner leftScaleEndTraj;
	
	TrajectoryPlanner rightScaleStartTraj;
	TrajectoryPlanner rightScaleFirstTurnTraj;
	TrajectoryPlanner rightScaleMidTraj;
	TrajectoryPlanner rightScaleEndTraj;

	
	TrajectoryPlanner driveForwardTraj;
	
	TrajectoryPlanner racetrackStartTraj;
	TrajectoryPlanner racetrackTurnTraj;
	
	final double FASTTOSLOW = 9.8;
	final double SLOWTOFAST = 29;
	
	double[][] rightSwitchAutoPlan = {{0,0,0},{100.5, -54.25 - 8, 0}};
	double[][] rightSwitchBackPlan = {{20, 5.25, 0}, {100.5, -54.25 - 8, 0}};
	
	double[][] leftSwitchAutoPlan = {{0,0,0},{100.5, 64.75 + 12, 0}};
	double[][] leftSwitchBackPlan = {{20, 5.25 + 5, 0}, {100.5, 64.75 + 12, 0}};
	
	double[][] switchGrabCubePlan = {{0,0,0}, {38.5 + 9, 0, 0}};
	
	double[][] leftScaleStartPlan = {{0,0,0},{160, 0, 0}};
	double[][] leftScaleEndPlan = {{160, 0,0},{260 + 12, -10 - 12, -45}};
	
	double[][] rightScaleStartPlan = {{0,0,0}, {160 + FASTTOSLOW, 0, 0}};
	double[][] rightScaleFirstTurnPlan = {{160-SLOWTOFAST, 0,0},{160, 0, 0}, {220, -60,-90}, {220, -60-SLOWTOFAST,-90}};
	double[][] rightScaleMidPlan = {{220, -60 + FASTTOSLOW, -90},{220, -171- FASTTOSLOW, -90}};
	double[][] rightScaleEndPlan = {{220, -171+SLOWTOFAST, -90},{220, -171, -90}, {260 + 12, -222.74 +12, 45}};	
	
	double[][] driveForwardPlan = {{0,0,0}, {90, 0, 0}};
	
	double[][] racetrackStartPlan = {{0, 0, 0}, {60+FASTTOSLOW+8.1, 0, 0}};
	double [][] racetrackTurnPlan = {{60-SLOWTOFAST, 0, 0}, {60,0,0}, {120, -60, -90}, {60, -120, -180}};

	boolean calibrate = false;
	boolean pidTune = false;
	
	boolean clamped = false;
	boolean gotReset = false;
	
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
		autonomousChooser.addObject("Ractrack", new RacetrackAuto(this));
		autonomousChooser.addObject("Drive Forward", new DriveForwardAuto(this));
		autonomousChooser.addObject("Switch Auto", new SwitchAuto(this));
		autonomousChooser.addObject("Scale Auto", new ScaleAuto(this));
		autonomousChooser.addObject("Do a Spin", new SpinnyAuto(this));
		SmartDashboard.putData("Autonomous Mode", autonomousChooser);
		
		CameraServer camera = CameraServer.getInstance();
		if(camera != null) {
			System.out.println("A camera was found");
			UsbCamera c = camera.startAutomaticCapture();
			if(c != null) {
				System.out.println("And it started.");
				c.setResolution(320, 180);
				c.setFPS(15);  /* Owen and Noelle found that A frame rate of 15 seems to work better than 29 */
			}
		}
	   
	   
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
		toggleShiftMode = new Button();
		elevatorUpButton = new Button();
		elevatorDownButton = new Button();
		
		testButton = new Button();
		elbowResetButton = new Button();
		nannyModeOffButton = new Button();
		
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
		
		//fourbarMotor = new TalonSRX(31);	//OWEN TODO CHANGED FROM TALONSPX TO VICTORSPX 3/17/18 FOR SHADOW ROBOT
		fourbarMotor = new VictorSPX(31);
		fourbarOutput = new FourbarOutput(this);
		
		elevatorMotor = new VictorSP(0 /*41*/);	// TODO HACKED TO REMOVE SPX FOR SHADOW
		elevatorOutput = new ElevatorOutput(this);
		
		lDriveEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		lDriveEncoder.setDistancePerPulse(1);
		rDriveEncoder = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
		rDriveEncoder.setDistancePerPulse(1);
		
		lElbowEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		rElbowEncoder = new Encoder(16, 17, false, Encoder.EncodingType.k4X);
		
		lPot = new AnalogPotentiometer(ai2, 360, lPotStart);
		rPot = new AnalogPotentiometer(ai1, 360, rPotStart);
		
		leftInches = new DriveInchesPIDSource(lDriveEncoder);
		rightInches = new DriveInchesPIDSource(rDriveEncoder);
		
		fourbarEncoder = new Encoder(8,9, false, Encoder.EncodingType.k4X);
		elevatorEncoder = new Encoder(6, 7, true, Encoder.EncodingType.k4X);

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
		
		gyro = new ADXRS450_Gyro();
		gyro.reset();
	
		lDriveOutput = new LeftDrivePIDOutput(this);
		rDriveOutput = new RightDrivePIDOutput(this);
		
		lIntakeOutput = new LeftIntakeOutput(this);
		rIntakeOutput = new RightIntakeOutput(this);
		
		rightSwitchAutoTraj = new TrajectoryPlanner(rightSwitchAutoPlan, 100*0.5, 300, 300, "RSwitchA"); //the integers are what the chassis is capable of, then we limit it with the decimals
		rightSwitchAutoTraj.generate();
		
		leftSwitchAutoTraj = new TrajectoryPlanner(leftSwitchAutoPlan, 100*0.5, 300, 300,"LSwitchA");
		leftSwitchAutoTraj.generate();
		
		leftSwitchBackTraj = new TrajectoryPlanner(leftSwitchBackPlan, 100*0.5, 300, 300, "LSwitchBack");
		leftSwitchBackTraj.generate();
		
		rightSwitchBackTraj = new TrajectoryPlanner(rightSwitchBackPlan, 100*0.5, 300, 300, "RSwitchBack");
		rightSwitchBackTraj.generate();
		
		switchGrabCubeTraj = new TrajectoryPlanner(switchGrabCubePlan, 100*0.5, 300, 300, "SwitchGrab");
		switchGrabCubeTraj.generate();
		
		leftScaleStartTraj = new TrajectoryPlanner(leftScaleStartPlan, 140, 600, 600, "LScaleStart");
		leftScaleStartTraj.generate();
		
		leftScaleEndTraj = new TrajectoryPlanner(leftScaleEndPlan, 100 *0.5, 300, 300, "LScaleEnd");
		leftScaleEndTraj.generate();
		
		rightScaleStartTraj = new TrajectoryPlanner(rightScaleStartPlan, 140, 600, 600,"RScaleStart");
		rightScaleStartTraj.generate();
		
		rightScaleFirstTurnTraj = new TrajectoryPlanner(rightScaleFirstTurnPlan, 100 *0.5, 300, 300,"RScaleFturn");
		rightScaleFirstTurnTraj.generate();
		
		rightScaleMidTraj = new TrajectoryPlanner(rightScaleMidPlan, 140, 600, 600, "RScaleMid");
		rightScaleMidTraj.generate();
		
		rightScaleEndTraj = new TrajectoryPlanner(rightScaleEndPlan, 100*0.5, 300, 300, "RScaleEnd");
		rightScaleEndTraj.generate();
		
		driveForwardTraj = new TrajectoryPlanner(driveForwardPlan, 100*.5, 300, 300, "DriveForward");
		driveForwardTraj.generate();
		
		racetrackStartTraj = new TrajectoryPlanner(racetrackStartPlan, 140, 600, 600, "RacetrackStart");
		racetrackStartTraj.generate();
		racetrackTurnTraj = new TrajectoryPlanner(racetrackTurnPlan, 100*.5, 300, 300, "RacetrackTurn");
		racetrackTurnTraj.generate();
		
		racetrackStartTraj.frankenstein(racetrackTurnTraj, 100*.5);
		
		leftControl = new SnazzyMotionPlanner(lowGearP, lowGearI, lowGearD, 0, lowGearKA, lowGearKV,  leftInches, lDriveOutput, 0.005, "Left.csv");
		rightControl= new SnazzyMotionPlanner(lowGearP, lowGearI, lowGearD, 0, lowGearKA, lowGearKV,  rightInches, rDriveOutput, 0.005,"Right.csv");
		
		leftPIDControl = new SnazzyPIDController(0.04, 0.001, 0.8, 0, leftInches, lDriveOutput, 0.005, "Left.csv");
		rightPIDControl= new SnazzyPIDController(0.04, 0.001, 0.8, 0, rightInches, rDriveOutput, 0.005,"Right.csv");
		
		fourbarPIDControl = new SnazzyPIDController(0.0005, 0.0, 0.0, 0.0, fourbarEncoder, fourbarOutput, 0.005, "FourbarPID.csv");
		fourbarPIDControl.setOutputRange(-0.8, 1.0);
		
		fourbarSetpoint = 0;
		//fourbarMotionControl = new SnazzyMotionPlanner(0.0005, 0.0, 0.0, 0.0, 0.00000310,0.00000387, fourbarEncoder, fourbarOutput, 0.005, "FourbarMotion.csv");
		
		elevatorPIDControl = new SnazzyPIDController(0.005, 0, 0, 0, elevatorEncoder, elevatorOutput, 0.005, "ElevatorPID.csv");
		//elevatorMotionControl = new SnazzyMotionPlanner(0, 0, 0, 0, 0, 0, elevatorEncoder, elevatorOutput, 0.005, "ElevatorMotion.csv");
		
		leftIntakeControl = new SnazzyPIDController(0.03, 0, 0, 0, lPot, lIntakeOutput, 0.005, "leftIntake.csv");
		rightIntakeControl = new SnazzyPIDController(0.03, 0, 0, 0, rPot, rIntakeOutput, 0.005, "rightIntake.csv");
		
		rIntakeSetpoint = rStart;
		lIntakeSetpoint = lStart;
		leftIntakeControl.setSetpoint(lIntakeSetpoint);
		rightIntakeControl.setSetpoint(rIntakeSetpoint);
		
		SmartDashboard.putNumber("P", 0.05);
		SmartDashboard.putNumber("I", 0.0);
		SmartDashboard.putNumber("D", 0.0);
		SmartDashboard.putNumber("Setpoint", 0);
		SmartDashboard.putNumber("L Encoder", 0);
		SmartDashboard.putNumber("R Encoder", 0);
		SmartDashboard.putNumber("L Elbow Enc", 0);
		SmartDashboard.putNumber("R Elbow Enc", 0);
		SmartDashboard.putNumber("L Elbow Pot", 0);
		SmartDashboard.putNumber("R Elbow Pot", 0);
		SmartDashboard.putString("Driving Gear", "Low");
		SmartDashboard.putString("Intake Setpoint", "Start");
		SmartDashboard.putNumber("Fourbar", 0);
		SmartDashboard.putNumber("Elevator", 0);
		SmartDashboard.putString("Clamped?", "Not Clamped");
		SmartDashboard.putString("ShiftMode", "Automatic");
		
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
		
		lIntakeSetpoint = lStart;
		leftIntakeControl.setSetpoint(lStart);
		rIntakeSetpoint = rStart;
		rightIntakeControl.setSetpoint(rStart);
		
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
			gearLowButton.update(driveStick.getRawButton(leftTrigger));
			gearHighButton.update(driveStick.getRawButton(leftBumper));
			intakeOpenButton.update(operatorStick.getRawButton(rightBumper) || driveStick.getRawButton(rightBumper));
			toggleShiftMode.update(driveStick.getRawButton(12));
			toggleIntakeDftButton.update(operatorStick.getRawButton(yButton)|| driveStick.getRawButton(yButton));
			intakeOutButton.update(operatorStick.getRawButton(rightTrigger) || driveStick.getRawButton(rightTrigger));
			unClampButton.update(operatorStick.getRawButton(bButton));
			testButton.update(driveStick.getRawButton(1));
			elbowResetButton.update(driveStick.getRawButton(startButton) || operatorStick.getRawButton(startButton));
			elevatorUpButton.update(operatorStick.getRawButton(leftBumper));
			elevatorDownButton.update(operatorStick.getRawButton(leftTrigger));
			nannyModeOffButton.update(driveStick.getRawButton(11));
		
			
			currentTime = Timer.getFPGATimestamp();
			rCurrentDist = rightInches.pidGet();
			lCurrentDist = leftInches.pidGet();
			lLastDistances.add(lCurrentDist);
			rLastDistances.add(rCurrentDist);
			lastTimes.add(currentTime);			

			if(!unClampButton.on()) {
				clamper.set(clampIt);
				clamped = true;
				if(fourbarEncoder.get() >= upperSafeZoneLimit && !gotReset) {
					toggleIntakeDftButton.reset();
					gotReset = true;
				}
	
				
			}else if(unClampButton.on()) {
				clamper.set(unClampIt);
				clamped = false;
				gotReset = false;
			}
			
			if(toggleShiftMode.on()) {
				manual = true;
				SmartDashboard.putString("ShiftMode", "Manual");
			} else {
				manual = false;
				SmartDashboard.putString("ShiftMode", "Automatic");
			}
			
			//Automatic Transmission
			if(!elevatorUp) {
				if(lLastDistances.size()==velocitySample && rLastDistances.size() == velocitySample ) {
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
				}
				
				if(manual) {
					if (gearLowButton.held()) {
						driveSolenoid.set(lowGear);
						SmartDashboard.putString("Driving Gear", "Low");
						
					}else {
						driveSolenoid.set(highGear);
						SmartDashboard.putString("Driving Gear", "High");
					}
				}
			}
			else {
				driveSolenoid.set(lowGear);
				SmartDashboard.putString("Driving Gear", "Low");
			}
			
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
			
			if(intakeOpenButton.changed()) {
				toggleIntakeDftButton.set();
			}
			
			if(getLElbow()>=(lStow - stowFudge) && getRElbow()<=(rStow+stowFudge) && fourbarEncoder.get()<= lowerSafeZoneLimit || fourbarEncoder.get()>= upperSafeZoneLimit) {
				if(toggleIntakeDftButton.on()) {
					stowDelay = 0.0;
					if(intakeOpenButton.held() && !intakeOutButton.held()) {
						rIntakeSetpoint = rOpen;
						lIntakeSetpoint = lOpen;
					} else if(!intakeOpenButton.held() && intakeOutButton.held()) {
						rIntakeSetpoint = rGrab;
						lIntakeSetpoint = lGrab;
						
					}else {
						rIntakeSetpoint = rGrab;
						lIntakeSetpoint = lGrab;
					}
					
				} else if(!toggleIntakeDftButton.on()) {
					if(intakeOpenButton.held() && !intakeOutButton.held()) {
						rIntakeSetpoint = rOpen;
						lIntakeSetpoint = lOpen;
						stowDelay = 0.0;
						
					} else if(!intakeOpenButton.held() && intakeOutButton.held()) {
						rIntakeSetpoint = rGrab;
						lIntakeSetpoint = lGrab;
						stowDelay = 0.0;
						
						
					}else {
						
						stowDelay += 1;
						if(Math.abs(getLElbow())<= lStow  && getRElbow()>= rStow) {
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
				lIntakeSetpoint = lClear;
				rIntakeSetpoint = rClear;
				unClampButton.reset();
				stowDelay = 0.0;
			}
			if(goinUp) {
				lIntakeSetpoint = lClear;
				rIntakeSetpoint = rClear;
				stowDelay = 0.0;
			}
			
			if(rIntakeSetpoint <= rGrab && !intakeOutButton.held() && !clamped) {
				leftBelt.set(ControlMode.PercentOutput, lBeltSpeed);
				rightBelt.set(ControlMode.PercentOutput, rBeltSpeed);
			} else if(intakeOutButton.held() && !clamped){
				leftBelt.set(ControlMode.PercentOutput, -lBeltSpeed);
				rightBelt.set(ControlMode.PercentOutput, -rBeltSpeed);
				
			}else {
				leftBelt.set(ControlMode.PercentOutput, 0.0);
				rightBelt.set(ControlMode.PercentOutput, 0.0);
			}
			
			leftIntakeControl.setSetpoint(lIntakeSetpoint);
			
			rightIntakeControl.setSetpoint(rIntakeSetpoint);
			
			if(fourbarIsSafe(fourbarSetpoint)) {
				if(elevatorUpButton.changed()) {
					elevatorPIDControl.setSetpoint(7800);
					elevatorUp = true;
					if(!nannyModeOffButton.on()) {
						maxPow = 0.5;
					}else {
						maxPow = 1.0;
						
					}
				} 
				if(elevatorDownButton.changed()) {
					elevatorPIDControl.setSetpoint(0);
					elevatorUp = false;
					maxPow = 1.0;
				}
			}
			
						
			SmartDashboard.putNumber("L Encoder", leftInches.pidGet());
			SmartDashboard.putNumber("R Encoder", rightInches.pidGet());
			SmartDashboard.putNumber("L Elbow Enc", getLElbow());
			SmartDashboard.putNumber("R Elbow Enc", getRElbow());
			SmartDashboard.putNumber("L Elbow Pot", lPot.get());
			SmartDashboard.putNumber("R Elbow Pot", rPot.get());
			SmartDashboard.putNumber("Fourbar", fourbarEncoder.get());
			SmartDashboard.putNumber("Elevator", elevatorEncoder.get());
			SmartDashboard.putBoolean("Nanny", elevatorUp);
			
			switch((int)rIntakeSetpoint) {
				case (int)rStart:
					intakeIndicator = "Start";
					break;
				case (int)rGrab:
					intakeIndicator = "Grab";
					break;
				case (int)rOpen:
					intakeIndicator = "Open";
					break;
				case (int) rClear:
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

			
			leftMotor1.set(ControlMode.PercentOutput, Math.pow(driveStick.getRawAxis(1), 3) * maxPow);
			leftMotor2.set(ControlMode.PercentOutput, Math.pow(driveStick.getRawAxis(1), 3) * maxPow);
			rightMotor3.set(ControlMode.PercentOutput, -Math.pow(driveStick.getRawAxis(3), 3) * maxPow);
			rightMotor4.set(ControlMode.PercentOutput, -Math.pow(driveStick.getRawAxis(3), 3) * maxPow);
			
			
			/*leftMotor1.set(ControlMode.PercentOutput, ());
			leftMotor2.set(ControlMode.PercentOutput,  driveStick.getRawAxis(1) + ());
			rightMotor3.set(ControlMode.PercentOutput,  -map(driveStick.getRawAxis(1) +(driveStick.getRawAxis(1) * driveStick.getRawAxis(2)), -2, 2, -1, 1));
			rightMotor4.set(ControlMode.PercentOutput, -map(driveStick.getRawAxis(1) +(driveStick.getRawAxis(1) * driveStick.getRawAxis(2)), -2, 2, -1, 1));
			*/			
		
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
		
		//driveSolenoid.set(lowGear);
		driveSolenoid.set(highGear);
		
		//leftControl.setkAkV(lowGearKA, lowGearKV);
		//rightControl.setkAkV(lowGearKA, lowGearKV);
		
		leftControl.setkAkV(highGearKA, highGearKV);
		rightControl.setkAkV(highGearKA, highGearKV);
		
		leftControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		rightControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//leftPIDControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//rightPIDControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//fourbarPIDControl.setPID(SmartDashboard.getNumber("P", 0),SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//leftIntakeControl.setPID(SmartDashboard.getNumber("P", 0.05), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//rightIntakeControl.setPID(SmartDashboard.getNumber("P", 0.05), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		//elevatorPIDControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		
		if(testButton.on()){
				if(testButton.changed()) {
					lDriveEncoder.reset();
					rDriveEncoder.reset();
					
					fourbarEncoder.reset();
					
					elevatorEncoder.reset();
					
					lElbowEncoder.reset();
					rElbowEncoder.reset();
					
					//leftPIDControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					//leftPIDControl.enable();
					
					//leftControl.configureTrajectory(driveForwardTraj.getLeftTrajectory(), false);
					//rightControl.configureTrajectory(driveForwardTraj.getRightTrajectory(), false);
					
					leftControl.configureTrajectory(driveForwardTraj.getLeftTrajectory(), false);
					rightControl.configureTrajectory(driveForwardTraj.getRightTrajectory(), false);
					
					
					//leftIntakeControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					//leftIntakeControl.enable();
					//rightIntakeControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					//rightIntakeControl.enable();
					
					//fourbarPIDControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					//fourbarPIDControl.enable();
					
					//rightPIDControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					//rightPIDControl.enable();
					
					//elevatorPIDControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					//elevatorPIDControl.enable();
					System.out.println("enable");
					
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
				//elevatorPIDControl.disable();
				
			}
		SmartDashboard.putNumber("L Encoder", leftInches.pidGet());
		SmartDashboard.putNumber("R Encoder", rightInches.pidGet());
		SmartDashboard.putNumber("Fourbar", fourbarEncoder.get());
		SmartDashboard.putNumber("L Elbow Enc", getLElbow());
		SmartDashboard.putNumber("R Elbow Enc", getRElbow());
		SmartDashboard.putNumber("L Elbow Pot", lPot.get());
		SmartDashboard.putNumber("R Elbow Pot", rPot.get());
		SmartDashboard.putNumber("Elevator", elevatorEncoder.get());
	}
	
	public boolean fourbarIsSafe(double futureSetpoint) {
		// 18000 safe for arm
		if(futureSetpoint >= upperSafeZoneLimit && fourbarEncoder.get() >= upperSafeZoneLimit) {
			return true;
		}
		if(Math.abs(getLElbow()-lStart)<= elbowFudge && Math.abs(getRElbow()-rStart)<=  elbowFudge) {
			return true;
		}
		if(Math.abs(Math.abs(getLElbow())-lClear)<= elbowFudge && Math.abs(Math.abs(getRElbow())-rClear)<= elbowFudge) {
			return true;
		}
		return false;
	}
	
	public void enableMechanismPIDs() {
		fourbarPIDControl.enable();
		leftIntakeControl.enable();
		rightIntakeControl.enable();
		elevatorPIDControl.enable();
	}
	
	public void disableMechanismPIDs() {
		fourbarPIDControl.disable();
		leftIntakeControl.disable();
		rightIntakeControl.disable();
		elevatorPIDControl.disable();
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
	
	public double map(double x, double in_min, double in_max, double out_min, double out_max)
	{
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
	public double getRElbow() {
		return rPot.get();
	}
	public double getLElbow() {
		return lPot.get();
	}
}

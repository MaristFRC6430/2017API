
package org.usfirst.frc.team6340.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import vision.GripPipeline;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team6340.robot.subsystems.ExampleSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive robotDrive;
	
	// Channels for the wheels
	final int kFrontLeftChannel = 2;
	final int kRearLeftChannel = 3;
	final int kFrontRightChannel = 1;
	final int kRearRightChannel = 0;

	Thread visionThread;
	
	public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
	public static OI oi;

	// The channel on the driver station that the joystick is connected to
	final static int kJoystickChannel = 0;
	
	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();
	
	static Joystick stick = new Joystick(kJoystickChannel);

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		//chooser.addDefault("Default Auto", new ExampleCommand());
		
		robotDrive = new RobotDrive(kFrontLeftChannel, kRearLeftChannel, kFrontRightChannel, kRearRightChannel);
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true); // invert the
																	// left side
																	// motors
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true); // you may need
																// to change or
																// remove this
																// to match your robot														// robot
		robotDrive.setExpiration(0.1);
		
		
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", chooser);
		
	
		visionThread = new Thread(() -> {
			GripPipeline pipe = new GripPipeline();
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			
			
			camera.setResolution(640, 480);
			camera.setFPS(10);
			
			CvSink cvSink = CameraServer.getInstance().getVideo();
			
			CvSource outputStream = CameraServer.getInstance().putVideo("Vision", 640, 480);
			
			Mat out = new Mat();
			Mat mat = new Mat();
			Mat edges = new Mat();
			int size = 0;
			 float avgX=0, avgY=0;
			
			while(!Thread.interrupted()) {
				if(cvSink.grabFrame(out) == 0) {
					
					outputStream.notifyError(cvSink.getError());
				continue;
				}
				
				Imgproc.resize(out, mat, new Size(320, 240));
				pipe.process(mat);
				Imgproc.Canny(pipe.hsvThresholdOutput(), edges, 0, 100);
				
					size=0;
					avgX=0;
				    avgY=0;
					
				for(int y = 0; y<edges.rows(); y++) {
					for(int x = 0; x<edges.cols(); x++) {
						if(edges.get(y, x)[0]>10) 
							{
						size++;
						avgX+=x;
						avgY+=y;
							}
					}
				}
				
				if(size>0) {
				avgX/=size;
				avgY/=size;
				}
								
				
				if(size>0) {
				Imgproc.circle(out, new Point(avgX*2,avgY*2), 20, new Scalar(0,0,255), 5);
				}
	
				outputStream.putFrame(out);
				
			}
			
		});
		visionThread.setDaemon(true);
		visionThread.start();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		robotDrive.setSafetyEnabled(true);
		while (isOperatorControl() && isEnabled()) {

			robotDrive.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getZ(), 0);

			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
	

	
}

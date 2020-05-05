//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


package API_ROS2_Sunrise;

// Configuration
import javax.inject.Inject;
import javax.inject.Named;

import org.apache.log4j.BasicConfigurator;


// Implementated classes
import API_ROS2_Sunrise.KMP_commander;
import API_ROS2_Sunrise.KMP_sensor_reader;
import API_ROS2_Sunrise.KMP_status_reader;
import API_ROS2_Sunrise.LBR_commander;
import API_ROS2_Sunrise.LBR_sensor_reader;
import API_ROS2_Sunrise.LBR_status_reader;
import API_ROS2_Sunrise.SafetyStateListener;

//RoboticsAPI
import com.kuka.roboticsAPI.annotations.*;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.deviceModel.LBR;

// DEBUG APPLICATION PAUSE:
import com.kuka.generated.ioAccess.ControlPanelIOGroup;

// AUT MODE: 3s, T1/T2/CRR: 2s
@ResumeAfterPauseEvent(delay = 0, afterRepositioning = false)
public class KMRiiwaSunriseApplication extends RoboticsAPIApplication{
	
	// Runtime Variables
	private volatile boolean AppRunning;
	private IAutomaticResumeFunction resumeFunction;
	SafetyStateListener safetylistener;
	
	// Declare KMP
	@Inject
	@Named("KMR_omniMove_200_1")
	public KmpOmniMove kmp;
	public Controller controller;
	
	// Declare LBR
	@Inject
	@Named("LBR_iiwa_14_R820_1")
	public LBR lbr;
	
	// TODO: declare tool
	//@Inject
	//@Named("name of tool")
	//public Tool tool;
	
	// Define UDP ports
	int KMP_status_port = 30001;
	int KMP_command_port = 30002;
	int KMP_laser_port = 30003;
	int KMP_odometry_port = 30004;
	int LBR_command_port = 30005;
	int LBR_status_port = 30006;
	int LBR_sensor_port = 30007;
	
	// Connection types
	String TCPConnection = "TCP";
	String UDPConnection = "UDP";

	// Implemented node classes
	KMP_commander kmp_commander;
	LBR_commander lbr_commander;
	LBR_sensor_reader lbr_sensor_reader;
	KMP_sensor_reader kmp_sensor_reader;
	KMP_status_reader kmp_status_reader;
	LBR_status_reader lbr_status_reader;

	// Close open UDP ports if not closed successfully
	CheckOpenPorts CheckPorts;
	
	// Check if application is paused:
	ControlPanelIOGroup ControlPanelIO;


	public void initialize() {
		System.out.println("Initializing Robotics API Application");
		// Check if any of the requested ports are open
		//CheckPorts = new CheckOpenPorts();
		//if(CheckPorts.run()){
		//	System.out.println("One or more of the Communication ports are open. Please restart the system by killing the process.");
//			dispose();
		//}
		// Configure application
		BasicConfigurator.configure();
		resumeFunction = getTaskFunction(IAutomaticResumeFunction.class);

		// Configure robot;
		controller = getController("KUKA_Sunrise_Cabinet_1");
		kmp = getContext().getDeviceFromType(KmpOmniMove.class);		
		lbr = getContext().getDeviceFromType(LBR.class);
		//tool.attachTo(lbr.getFlange());
		
		// ControlPanelIO:
		ControlPanelIO = new ControlPanelIOGroup(controller);

		// Create nodes for communication
		kmp_commander = new KMP_commander(KMP_command_port, kmp, TCPConnection);
		lbr_commander = new LBR_commander(LBR_command_port, lbr, TCPConnection, getApplicationData().getFrame("/DrivePos"));

		
		// SafetyStateListener
		safetylistener = new SafetyStateListener(controller, lbr_commander,kmp_commander,lbr_status_reader,kmp_status_reader);
		safetylistener.startSafetyStateListener();
		
		
		// Check if a commander node is active
		long startTime = System.currentTimeMillis();
		int shutDownAfterMs = 7000; 
		while(!AppRunning) {
			if(kmp_commander.isSocketConnected() || lbr_commander.isSocketConnected()){
					AppRunning = true;
					System.out.println("Application ready to run!");	
					break;
			}else if((System.currentTimeMillis() - startTime) > shutDownAfterMs){
				System.out.println("Could not connect to a command node after " + shutDownAfterMs/1000 + "s. Shutting down.");	
				shutdown_application();
				break;
			}				
		}
		
		if(AppRunning){
			kmp_status_reader = new KMP_status_reader(KMP_status_port, kmp,TCPConnection, controller);
			lbr_status_reader = new LBR_status_reader(LBR_status_port, lbr,TCPConnection);
			lbr_sensor_reader = new LBR_sensor_reader(LBR_sensor_port,lbr, TCPConnection);
			kmp_sensor_reader = new KMP_sensor_reader(KMP_laser_port, KMP_odometry_port, TCPConnection, TCPConnection);
		}
	}
	

	public void shutdown_application(){
		System.out.println("----- Shutting down Application -----");
		lbr_commander.closed=true;
		kmp_commander.closed=true;

		kmp_commander.close();
		lbr_commander.close();

		kmp_status_reader.close();
		lbr_status_reader.close();
		
		lbr_sensor_reader.close();
		kmp_sensor_reader.close();

    	System.out.println("Application terminated");
    	    	
    	try{
    		dispose();
    	}catch(Exception e){
    		System.out.println("Application could not be terminated cleanly: " + e);
    	}
    	}
	
	public void run() {
		setAutomaticallyResumable(true);

		System.out.println("Running app!");
		
//		Start all connected nodes
		kmp_commander.setPriority(Thread.MAX_PRIORITY);
		lbr_commander.setPriority(Thread.MAX_PRIORITY);
		if(kmp_commander.isSocketConnected()) {
			kmp_commander.start();
		}
		if(kmp_status_reader.isSocketConnected()) {
			kmp_status_reader.start();
		}
		
		if(lbr_commander.isSocketConnected()) {
			lbr_commander.start();
		
		}
		if(lbr_status_reader.isSocketConnected()) {
			lbr_status_reader.start();		
		}

		if(lbr_sensor_reader.isSocketConnected()) {
			lbr_sensor_reader.start();
		}
		
		if(kmp_sensor_reader.isSocketConnected()) {
			kmp_sensor_reader.start();
		}

		
	
		while(AppRunning)
		{    

			AppRunning = (!(kmp_commander.getShutdown() || lbr_commander.getShutdown()));
		}
		System.out.println("Shutdown message received from ROS");
		shutdown_application();
	}
	
	private void setAutomaticallyResumable(boolean enable)
	{
		if(enable)
		{
			resumeFunction.enableApplicationResuming(getClass().getCanonicalName());
			return;
		}
		resumeFunction.disableApplicationResuming(getClass().getCanonicalName());		
	}

	
	public static void main(String[] args){
		KMRiiwaSunriseApplication app = new KMRiiwaSunriseApplication();
		app.runApplication();
	}
	
}

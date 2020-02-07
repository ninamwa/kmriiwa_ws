
package testwithrobot;

// Configuration
import javax.inject.Inject;
import javax.inject.Named;
import org.apache.log4j.BasicConfigurator;

// Socket
import java.net.InetSocketAddress;

// Implementated classes
import testwithrobot.KMP_commander;
import testwithrobot.KMP_sensor_reader;
import testwithrobot.KMP_status_reader;
import testwithrobot.LBR_commander;
import testwithrobot.LBR_sensor_reader;
import testwithrobot.LBR_status_reader;


// RoboticsAPI
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.deviceModel.LBR;

public class API_ROS2_Sunrise extends RoboticsAPIApplication{
	
	private volatile boolean AppRunning;


	// Declare KMP
	@Inject
	@Named("KMR_200_2")
	public KmpOmniMove kmp;
	public Controller controller;
	
	// Declare LBR
	@Inject
	@Named("LBR_iiwa_14_R820_1")
	public LBR lbr;
	
	// Declare tool
// TODO:
	//@Inject
	//@Named("name of tool")
	//public Tool tool;
	
	// Define UDP ports
	int KMP_status_port = 30001;
	int KMP_command_port = 30008;
	int KMP_laser_port = 30003;
	int KMP_odometry_port = 30004;
	int LBR_command_port = 30005;
	int LBR_status_port = 30006;
	int LBR_sensor_port = 30007;

	String TCPConnection = "TCP";
	String UDPConnection = "UDP";

	// Implemented thread classes
	KMP_commander kmp_commander;
	LBR_commander lbr_commander;
	LBR_sensor_reader lbr_sensor_reader;
	KMP_sensor_reader kmp_sensor_reader;
	KMP_status_reader kmp_status_reader;
	LBR_status_reader lbr_status_reader;

	// Close open UDP ports if not closed successfully
	CheckOpenPorts CheckPorts;
	boolean OpenPorts;

	public void initialize() {
		// Check if any of the requested ports are open, and close if any
		CheckPorts = new CheckOpenPorts();
		OpenPorts = CheckPorts.run();
		if(OpenPorts){
			System.out.println("One or more of the Communication ports are open. Please restart the system by killing the process.");
//			dispose();
		}
		// Configure application
		BasicConfigurator.configure();
		System.out.println("Initializing Robotics API Application");

		// Configure robot;
		controller = getController("KUKA_Sunrise_Cabinet_1");
		kmp = getContext().getDeviceFromType(KmpOmniMove.class);		
		lbr = getContext().getDeviceFromType(LBR.class);
		//tool.attachTo(lbr.getFlange());
		
		// Create nodes for communication
		kmp_commander = new KMP_commander(KMP_command_port, kmp, UDPConnection);
		//lbr_commander = new LBR_commander(LBR_command_port, lbr, UDPConnection);
		kmp_status_reader = new KMP_status_reader(KMP_status_port, kmp, UDPConnection, controller);
		//lbr_status_reader = new LBR_status_reader(LBR_status_port, lbr,UDPConnection);
		//lbr_sensor_reader = new LBR_sensor_reader(LBR_sensor_port,lbr, UDPConnection);
		kmp_sensor_reader = new KMP_sensor_reader(KMP_laser_port, KMP_odometry_port, UDPConnection, UDPConnection);
		
		// Check if a commander node is active
		long startTime = System.currentTimeMillis();
		int shutDownAfterMs = 7000; 
		while(!AppRunning) {
			kmp_commander.setLBRConnected(false);
			//kmp_commander.setLBRConnected(lbr_commander.isSocketConnected());
			//lbr_commander.setKMPConnected(kmp_commander.isSocketConnected());
			if(kmp_commander.isSocketConnected()){// || lbr_commander.isSocketConnected()){
					AppRunning = true;
					System.out.println("Application ready to run!");	
					break;
			}else if((System.currentTimeMillis() - startTime) > shutDownAfterMs){
				shutdown_application();
				System.out.println("Could not connect to a command node after " + shutDownAfterMs/1000 + "s. Shutting down.");	
				break;
			}				
		}
	}
	
	public void shutdown_application(){
		System.out.println("----- Shutting down Application -----");
		
		kmp_commander.close();
		//lbr_commander.close();

		kmp_status_reader.close();
		//lbr_status_reader.close();
		
		//lbr_sensor_reader.close();
		kmp_sensor_reader.close();

    	System.out.println("Application terminated");
    	    	
    	try{
    		dispose();
    	}catch(Exception e){
    		System.out.println("Application could not be terminated cleanly: " + e);
    	}
    	}
	
	public void run() {
		System.out.println("Running app!");
		// Start all connected nodes
		if(kmp_commander.isSocketConnected()) {
			kmp_commander.start();
		}
		if(kmp_status_reader.isSocketConnected()) {
			kmp_status_reader.start();
		}
		
		//if(lbr_commander.isSocketConnected()) {
		//	lbr_commander.start();
		
		//}
		
//		if(lbr_status_reader.isSocketConnected()) {
//			lbr_status_reader.start();		
//		}

		//if(lbr_sensor_reader.isRequested()) {
		//	lbr_sensor_reader.start();
		//}
		
		if(kmp_sensor_reader.isRequested()) {
			kmp_sensor_reader.start();
		}
		while(AppRunning)
		{    
			AppRunning = (!(kmp_commander.getShutdown())); // || lbr_commander.getShutdown()
		}
		System.out.println("Shutdown message received from ROS");
		shutdown_application();
	}

	public static void main(String[] args){
		API_ROS2_Sunrise app = new API_ROS2_Sunrise();
		app.runApplication();
	}
	
}

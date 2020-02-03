
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
	
	// Declare KMR and Sunrise Cabinet
	@Inject
	@Named("KMR_200_2")
	public KmpOmniMove kmp;
	
	public Controller controller;
	
	@Inject
	@Named("LBR_iiwa_14_R820_1")
	public LBR lbr;
	
	// TODO: trenger vi tool?
	//@Inject
	//@Named("name of tool")
	//public Tool tool;
	
	// Define UDP ports
	int LBR_command_port = 30001;
	int LBR_status_port = 30003;
	int LBR_sensor_port = 30004;
	
	int KMP_status_port = 30001;
	int KMP_command_port = 30002;
	int KMP_laser_port = 30003;
	int KMP_odometry_port = 30004;
	
	String TCPConnection = "TCP";
	String UDPConnection = "UDP";

    

	// Implemented thread classes
	KMP_commander kmp_commander;
	LBR_commander lbr_commander;
	LBR_sensor_reader lbr_sensor_reader;
	KMP_sensor_reader kmp_sensor_reader;
	KMP_status_reader kmp_status_reader;
	LBR_status_reader lbr_status_reader;

	public void initialize() {
		
		BasicConfigurator.configure();
		getLogger().info("Initializing Application");

		controller = getController("KUKA_Sunrise_Cabinet_1");
		kmp = getContext().getDeviceFromType(KmpOmniMove.class);		
		lbr = getContext().getDeviceFromType(LBR.class);
		//tool.attachTo(lbr.getFlange());
		
		kmp_commander = new KMP_commander(KMP_command_port, kmp, UDPConnection);
		//lbr_commander = new LBR_commander(LBR_command_port, lbr, UDPConnection);
		kmp_status_reader = new KMP_status_reader(KMP_status_port, kmp, UDPConnection);
		//lbr_status_reader = new LBR_status_reader(LBR_status_port, lbr,UDPConnection);
		//lbr_sensor_reader = new LBR_sensor_reader(LBR_sensor_port,lbr, UDPConnection);
		kmp_sensor_reader = new KMP_sensor_reader(KMP_laser_port, KMP_odometry_port, UDPConnection, UDPConnection);
		int number_of_tries = 0;
		while(!AppRunning) {
			if(ConnectionsKMP() || ConnectionsLBR()) {
					AppRunning = true;
					break;
			}else {
				if(!(kmp_commander.isSocketConnected())) {
					kmp_commander.createSocket();
					number_of_tries = number_of_tries +1;
					if (number_of_tries == 5){
						shutdown_application();
						break;
						
					}
				}
				//if(!(kmp_status_reader.isSocketConnected())) {
				//	kmp_status_reader.createSocket();
				//}
				//lbr_commander.createSocket();
				//lbr_status_reader.createSocket();
			}
		}
		System.out.println("Application ready to run!");	
	}
	
	// TODO: kan slåes sammen til en metode, med OR operator, når vi faktisk har LBR oppe og går 
		public boolean ConnectionsKMP() {
			return (kmp_commander.isSocketConnected());// && kmp_status_reader.isSocketConnected()); 
		}
		public boolean ConnectionsLBR() {
			return false;
			//return (lbr_commander.isSocketConnected() && lbr_status_reader.isSocketConnected());
		}
	
	public void shutdown_application(){
		System.out.println("----- Shutting down Application -----");
		
		kmp_commander.close();
		//lbr_commander.close();

		kmp_status_reader.close();
		//lbr_status_reader.close();
		
		//lbr_sensor_reader.close();
		kmp_sensor_reader.close();

    	System.out.println("Disconnected!");
    	
    	// TODO: END THREADS?
    	
    	try{
    		dispose();
    	}catch(Exception e){
    		System.out.println("Application could not be terminated cleanly: " + e);
    	}
    	}
	
	
	
	public void run() {
		System.out.println("Running app!");
		if(ConnectionsKMP()) {
			kmp_commander.start();
			kmp_status_reader.start();
		}
		
		//if(ConnectionsLBR()) {
		//	lbr_commander.start();
		//	lbr_status_reader.start();
		//}
		
		//if(lbr_sensor_reader.isRequested()) {
		//	lbr_sensor_reader.start();
		//}
		
		// TODO: Må vi ha no mer sjekk på tilkobling?
		// Dette er en stor feilkilde hvis tilkobling feiler
		if(kmp_sensor_reader.isRequested()) {
			kmp_sensor_reader.start();
		}
		while(AppRunning)
		{    
			AppRunning = (!(kmp_commander.getShutdown())); // || lbr_commander.getShutdown()
		}
		System.out.println("Shutdown message received from ROS");
		shutdown_application();
		System.out.println("- - - APPLICATION TERMINATED - - -");
	}

	public static void main(String[] args){
		API_ROS2_Sunrise app = new API_ROS2_Sunrise();
		app.runApplication();
	}
	
	
	

}

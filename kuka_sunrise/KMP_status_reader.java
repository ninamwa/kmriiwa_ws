package testwithrobot;


//Implemented classes
import java.util.concurrent.TimeUnit;

import testwithrobot.UDPSocket;
import testwithrobot.TCPSocket;
import testwithrobot.ISocket;

//RoboticsAPI
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.generated.ioAccess.MobilePlatformStateSignalsIOGroup;
import com.kuka.generated.ioAccess.ScannerSignalsIOGroup;
import com.kuka.roboticsAPI.deviceModel.OperationMode;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
//TODO: importere alle klasser fra SunriseOmniMoveMobilePlatform, scannerIO
public class KMP_status_reader extends Thread{
	
	int port;
	ISocket socket;
	String ConnectionType;
	public volatile boolean closed = false;
	
	ScannerSignalsIOGroup scannerIOGroup;
	MobilePlatformStateSignalsIOGroup MPStateSignalsIOGroup;
	
	KmpOmniMove kmp;
	private OperationMode operation_mode = null;
	private Object isReadyToMove = null; //IsReadyToMove() 
//	private double[] maximumVelocities; //GetMaximumVelocities
	private volatile boolean WarningField = false;
	private volatile boolean ProtectionField = false;
//	TODO: SKAL VI HA EN SJEKK P� DENNE HER? IFT LESE PROT OG WARNING FIELD?
	
	public KMP_status_reader(int UDPport, KmpOmniMove robot,String ConnectionType, Controller controller) {
		this.port = UDPport;
		this.kmp = robot;
		this.ConnectionType = ConnectionType;
		createSocket();
		this.scannerIOGroup = new ScannerSignalsIOGroup(controller);
		
		if (!(isSocketConnected())) {
			System.out.println("Starting thread to connect KMP status node....");
			Thread monitorKMPStatusConnections = new MonitorKMPStatusConnectionsThread();
			monitorKMPStatusConnections.start();
			}
	}
	
	public void createSocket(){
		if (this.ConnectionType == "TCP") {
			 this.socket = new TCPSocket(this.port);
		}
		else {
			this.socket = new UDPSocket(this.port);
		}
	}
	public void run() {
		while(isSocketConnected() && (!(closed)))
		{	
//FIND OUT HOW MUCH TO SLEEP. SAMME RATE SOM ODOMETRY?
			updateOperationMode();
			updateReadyToMove();
			updateWarningFieldState();
			updateProtectionFieldState();
			sendStatus();
			
			if(!isSocketConnected() || (closed)){
				break;
			}
			try {
				TimeUnit.MILLISECONDS.sleep(30);
			} catch (InterruptedException e) {
				System.out.println("KMP status thread could not sleep");
			}
		}
 }

	
	private void updateOperationMode() {
		this.operation_mode = kmp.getOperationMode();
	}
	
	private void updateReadyToMove() {
		this.isReadyToMove = kmp.isReadyToMove();
	}
	
	// scannerIO
	private void updateWarningFieldState() {
//		signalnames =  Arrays.asList("WarningField_B1", "WarningField_B4", "WarningFieldComplete");
		if(!(closed)){
			try{
				this.WarningField = this.scannerIOGroup.getWarningFieldComplete(); // 
			}catch(Exception e){
//					System.out.println("Could not read warning field: " + e);
				}
			}
		}
	
	// scannerIO
	private void updateProtectionFieldState() {
//		signalnames =  Arrays.asList("WarningField_B1", "WarningField_B4", "WarningFieldComplete");
		if(!(closed)){
			try{
				this.ProtectionField = this.scannerIOGroup.getProtectionFieldComplete();
			}catch(Exception e){
//				System.out.println("Could not read protection field: " + e);
				}
		}
		}

	
	// TODO: Må se på output på de forskjellige. F.eks. list, må disse hentes ut separat?
	private String generateStatusString() {
		return 	">kmp_statusdata ,"  + System.nanoTime() + ",OperationMode:"+ this.operation_mode.toString() + ",ReadyToMove:" + this.isReadyToMove + ",WarningField:" + this.WarningField + ",ProtectionField:" + this.ProtectionField;
	}
	
	public void sendStatus() {
		String toSend = this.generateStatusString();
		if(isSocketConnected() && (!(closed))){
			try{
				this.socket.send_message(toSend);
				if(closed){
					System.out.println("STATUS POSTER SELV OM HAN IKKE F�R LOV :D");
				}
			}catch(Exception e){
				System.out.println("Could not send Operation mode to ROS: " + e);
			}
		}
	}
	
	public class MonitorKMPStatusConnectionsThread extends Thread {
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				
				createSocket();
				if (isSocketConnected()){
					break;
				}
				try {
					Thread.sleep(5000);
				} catch (InterruptedException e) {
					System.out.println("");
				}
				
			}
			if(!closed){
				System.out.println("Connection with KMP Status Node OK!");
				runmainthread();					
				}	
		}
	}
	public void runmainthread(){
		this.run();
	}

	public void close() {
		closed = true;
		socket.close();
	}
	
	public boolean isSocketConnected() {
		return socket.isConnected();
	}
	public boolean isSocketCreated() {
		return !(socket==null);
	}

}
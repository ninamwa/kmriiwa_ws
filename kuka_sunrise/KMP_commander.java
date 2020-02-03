package testwithrobot;

// Implemented classes
import testwithrobot.KMPjogger;
import testwithrobot.UDPSocket;
import testwithrobot.TCPSocket;
import testwithrobot.ISocket;


// RoboticsAPI
import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;




public class KMP_commander extends Thread{
	
	public volatile boolean shutdown;
	int port;
	ISocket socket;
	String ConnectionType;

	KMPjogger kmp_jogger;
	KmpOmniMove kmp;
	
	// Motion variables: KMP
	ICommandContainer KMP_currentMotion;
	double[] velocities = new double[3];
	boolean KMP_is_Moving;
	
	public KMP_commander(int UDPport, KmpOmniMove robot, String ConnectionType) {
		this.port=UDPport;
		this.kmp = robot;
		this.kmp_jogger = new KMPjogger((ICartesianJoggingSupport)kmp);
		this.shutdown = false;
		this.ConnectionType = ConnectionType;
		createSocket();
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
		while(isSocketConnected())
		{
			String Commandstr =this.socket.receive_message(); 
	    	String []splt = Commandstr.split(" ");
	    	if ((splt[0]).equals("shutdown")){
				this.shutdown = true;	
				break;
				}
			if ((splt[0]).equals("setTwist")){
				setNewVelocity(Commandstr);
				}
		}
    }
	
	
	public void setNewVelocity(String vel){
		String []lineSplt = vel.split(" ");
		if (lineSplt.length==4){
			this.velocities[0] =Double.parseDouble(lineSplt[1]); // x
			this.velocities[1] = Double.parseDouble(lineSplt[2]); // y
			this.velocities[2] = Double.parseDouble(lineSplt[3]);  // theta
		}
		if(velocities[0] !=0 ||velocities[1] !=0 ||velocities[2] !=0 ) {
			  if(KMP_is_Moving) {
				  this.kmp_jogger.updateVelocities(this.velocities);
			  }
			  else {
				  this.kmp_jogger.updateVelocities(this.velocities);
				  this.kmp_jogger.startJoggingExecution();
				  KMP_is_Moving = true;
			  }
		  }else {
			  if(KMP_is_Moving) {
				  this.kmp_jogger.killJoggingExecution();
				  KMP_is_Moving=false;
			  }
		  }
	}
	
	public boolean isKmpMoving() {
		return KMP_is_Moving;
	}
	
	public boolean getShutdown() {
		return this.shutdown;
	}
	public void close() {
		setNewVelocity("vel 0 0 0");
		try {
			kmp_jogger.killJoggingExecution();
		}catch(Exception e){
			System.out.println("Could not kill jogging execution");
		}
		try{
			this.socket.close();
			}catch(Exception e){
				System.out.println("Could not close KMP commander connection: " +e);
			}
	}
	
	public boolean isSocketConnected() {
		return this.socket.isConnected();
	}
	
	public boolean isSocketCreated() {
		return !(this.socket==null);
	}
}

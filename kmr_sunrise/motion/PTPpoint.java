package API_ROS2_Sunrise;


import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.motionModel.PTP;


public class PTPpoint{
	PTP ptppoint;
	String pointtype;
	
	public PTPpoint(String type, JointPosition jp, double[] vel, double[] acc){
		ptppoint = new PTP(jp);
		ptppoint.setJointVelocityRel(vel);
		ptppoint.setJointAccelerationRel(acc);
		pointtype = type;
	}
	
	
	public PTP getPTP(){
		return ptppoint;
	}
	
	public String getPointType(){
		return pointtype;
	}
	

	
	}

// Copyright 2019 Nina Marie Wahl og Charlotte Heggem.
// Copyright 2019 Norwegian University of Science and Technology.
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


import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.SplineMotionJP;


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


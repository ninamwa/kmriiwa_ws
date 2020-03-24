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


import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.DispatchedEventData;
import com.kuka.roboticsAPI.controllerModel.StatePortData;
import com.kuka.roboticsAPI.controllerModel.sunrise.ISunriseControllerStateListener;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState;
import com.kuka.roboticsAPI.deviceModel.Device;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;

public class SafetyStateListener {
	Controller controller;
	LBR_commander lbr_commander;
	KMP_commander kmp_commander;
	LBR_status_reader lbr_status_reader;
	KMP_status_reader kmp_status_reader;
	
	public SafetyStateListener(Controller cont, LBR_commander lbr, KMP_commander kmp,LBR_status_reader lbr_status, KMP_status_reader kmp_status) {
		controller = cont;
		lbr_commander = lbr;
		kmp_commander = kmp;
		lbr_status_reader = lbr_status;
		kmp_status_reader = kmp_status;
		
	}
	public void startSafetyStateListener() {
		controller.addControllerListener( new ISunriseControllerStateListener(){

			@Override
			public void onSafetyStateChanged(Device device,
					SunriseSafetyState safetyState) {
				if(safetyState.getSafetyStopSignal()==SunriseSafetyState.SafetyStopType.STOP1){
					kmp_commander.setEmergencyStop(true);
					lbr_commander.setEmergencyStop(true);
					lbr_status_reader.setLBRemergencyStop(true);
					kmp_status_reader.setKMPemergencyStop(true);
					// TODO: Add status listener if this works!
				}else if(safetyState.getSafetyStopSignal()==SunriseSafetyState.SafetyStopType.NOSTOP) {

					kmp_commander.setEmergencyStop(false);
					lbr_commander.setEmergencyStop(false);
					lbr_status_reader.setLBRemergencyStop(false);
					kmp_status_reader.setKMPemergencyStop(false);
				}
			
			}

			@Override
			public void onFieldBusDeviceConfigurationChangeReceived(
					String controllerName, DispatchedEventData dispatchedEvent) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onFieldBusDeviceIdentificationRequestReceived(
					String controllerName, DispatchedEventData dispatchedEvent) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onIsReadyToMoveChanged(Device device,
					boolean isReadyToMove) {
				if(device.getClass()==KmpOmniMove.class){
					System.out.println("KMP IS READY TO MOVE: " + isReadyToMove);
				}
			}

			@Override
			public void onShutdown(Controller controller) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onStatePortChangeReceived(Controller controller,
					StatePortData statePort) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onConnectionLost(Controller controller) {
				// TODO Auto-generated method stub
				
			}


		});
	}
}

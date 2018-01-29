/** @file
@brief OSVR plugin for LYRobotix Nolo trackers

@date 2018

@author
Nanospork
<http://www.nanospork.com>
*/

// Copyright 2014 Sensics, Inc.
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

// Internal Includes
#include <osvr/PluginKit/PluginKit.h>
#include <osvr/PluginKit/AnalogInterfaceC.h>
#include <osvr/PluginKit/ButtonInterfaceC.h>
#include <osvr/PluginKit/TrackerInterfaceC.h>

// Generated JSON header file
#include "com_osvr_Nolo_json.h"

// Library/third-party includes
#include <nolo_api.h>

// Standard includes
#include <iostream>
#include <wchar.h>
#include <string.h>

// Anonymous namespace to avoid symbol collision
namespace {

	const static int NUM_AXIS = 4;
	const static int NUM_BUTTONS = 6;
	class NoloDevice {
	public:
		NoloDevice(OSVR_PluginRegContext ctx) {
			/// Create the initialization options
			OSVR_DeviceInitOptions opts = osvrDeviceCreateInitOptions(ctx);

			std::cout << "Starting nolo-osvr driver.\n";

			m_is_nolo_connected = false;
			m_new_data_available = false;
			//changeme
			//std::cout << "Nolo: opening " << path <<
			//	" for " << this << std::endl;

			/// Indicate that we'll want 7 analog channels.
			//osvrDeviceAnalogConfigure(opts, &m_analog, 2*3+1);
			// update this to 9 analog channels to include the trigger
			osvrDeviceAnalogConfigure(opts, &m_analog, NUM_AXIS * 2 + 1);
			/// And 6 buttons per controller
			osvrDeviceButtonConfigure(opts, &m_button, 2 * NUM_BUTTONS);
			/// And tracker capability
			osvrDeviceTrackerConfigure(opts, &m_tracker);

			/// Create the device token with the options
			m_dev.initAsync(ctx, "LYRobotix Nolo", opts);

			/// Send JSON descriptor
			m_dev.sendJsonDescriptor(com_osvr_Nolo_json);

			/// Register update callback
			m_dev.registerUpdateCallback(this);

			NOLO::registerDisConnectCallBack(disconnectNolo, this);
			NOLO::registerConnectSuccessCallBack(connectNolo, this);
			NOLO::registerExpandDataNotifyCallBack(expandDataNotify, this);

			// Enable this below for ceiling mode
			//std::cout << "Nolo: Set CeilingMode" << std::endl; 
			//NOLO::set_Nolo_PlayMode(NOLO::CeilingMode);

			// Switch to polling structure to avoid strange SteamVR bug.
			NOLO::registerNoloDataNotifyCallBack(noloDataNotify, this);
			NOLO::search_Nolo_Device();

		}
		~NoloDevice() {
			std::cout << "Nolo: deleting " << this << std::endl;
			NOLO::close_Nolo_Device();
		}
		OSVR_ReturnCode update() {
			// Switch to polling structure to avoid strange SteamVR bug.
			if (m_new_data_available) {
				m_new_data_available = false;
				readNewData(NOLO::get_Nolo_NoloData(), this);
			}

			return OSVR_RETURN_SUCCESS;
		}

		static void disconnectNolo(void* context) {
			NoloDevice& device = *(NoloDevice*)context;
			device.m_is_nolo_connected = false;
			std::cout << "Nolo hardware disconnected.\n";
		}

		static void connectNolo(void* context) {
			NoloDevice& device = *(NoloDevice*)context;
			device.m_is_nolo_connected = true;
			std::cout << "Nolo hardware connected.\n";
		}

		static void expandDataNotify(NOLO::ExpandMsgType msgType, void* context) {
			NoloDevice& device = *(NoloDevice*)context;
		}

		static void noloDataNotify(NOLO::NoloData data, void* context) {
			NoloDevice& device = *(NoloDevice*)context;
			device.m_new_data_available = true;
		}

		static void readNewData(NOLO::NoloData data, void* context) {
			NoloDevice& device = *(NoloDevice*)context;
			if (!device.m_is_nolo_connected) {
				return;
			}

			osvrTimeValueGetNow(&device.m_lastreport_time);
			//static int i = 0;
			//if (i > 1000) {
			//	std::cout << device.m_lastreport_time.seconds << " " << device.m_lastreport_time.microseconds << "\n";
			//	std::cout << data.right_Controller_Data.vecVelocity.x << "," << data.right_Controller_Data.vecVelocity.y << "," << data.right_Controller_Data.vecVelocity.z << "\n";
			//	std::cout << data.left_Controller_Data.vecVelocity.x << "," << data.left_Controller_Data.vecVelocity.y << "," << data.left_Controller_Data.vecVelocity.z << "\n";
			//	std::cout.flush();
			//	i = 0;
			//}
			//++i;

			double translationScale = 1.0f;

			/*
			Report HMD Pose
			*/
			OSVR_PositionState home;
			OSVR_PoseState hmd;
			OSVR_VelocityState hmdVel;
			OSVR_AccelerationState hmdAcc;

			device.SetPosition(home, data.hmdData.HMDInitPosition);
			device.SetPose(hmd, data.hmdData.HMDPosition, data.hmdData.HMDRotation);
			device.SetVelocity(hmdVel, data.hmdData.vecVelocity, data.hmdData.vecAngularVelocity);
			device.SetAcceleration(hmdAcc, data.hmdData.vecAcceleration, data.hmdData.vecAngularAcceleration);

			osvrDeviceTrackerSendPositionTimestamped(device.m_dev, device.m_tracker, &home, 0, &device.m_lastreport_time);
			osvrDeviceTrackerSendPoseTimestamped(device.m_dev, device.m_tracker, &hmd, 1, &device.m_lastreport_time);
			osvrDeviceTrackerSendVelocityTimestamped(device.m_dev, device.m_tracker, &hmdVel, 1, &device.m_lastreport_time);
			osvrDeviceTrackerSendAccelerationTimestamped(device.m_dev, device.m_tracker, &hmdAcc, 1, &device.m_lastreport_time);

			///*
			//Report Controller Pose
			//*/
			OSVR_PoseState leftController;
			OSVR_VelocityState leftVel;
			OSVR_AccelerationState leftAcc;

			OSVR_PoseState rightController;
			OSVR_VelocityState rightVel;
			OSVR_AccelerationState rightAcc;

			device.SetPose(leftController, data.left_Controller_Data.ControllerPosition, data.left_Controller_Data.ControllerRotation);
			device.SetVelocity(leftVel, data.left_Controller_Data.vecVelocity, data.left_Controller_Data.vecAngularVelocity);
			device.SetAcceleration(leftAcc, data.left_Controller_Data.vecAcceleration, data.left_Controller_Data.vecAngularAcceleration);

			osvrDeviceTrackerSendPoseTimestamped(device.m_dev, device.m_tracker, &leftController, 2, &device.m_lastreport_time);
			osvrDeviceTrackerSendVelocityTimestamped(device.m_dev, device.m_tracker, &leftVel, 2, &device.m_lastreport_time);
			osvrDeviceTrackerSendAccelerationTimestamped(device.m_dev, device.m_tracker, &leftAcc, 2, &device.m_lastreport_time);

			device.SetPose(rightController, data.right_Controller_Data.ControllerPosition, data.right_Controller_Data.ControllerRotation);
			device.SetVelocity(rightVel, data.right_Controller_Data.vecVelocity, data.right_Controller_Data.vecAngularVelocity);
			device.SetAcceleration(rightAcc, data.right_Controller_Data.vecAcceleration, data.right_Controller_Data.vecAngularAcceleration);

			osvrDeviceTrackerSendPoseTimestamped(device.m_dev, device.m_tracker, &rightController, 3, &device.m_lastreport_time);
			osvrDeviceTrackerSendVelocityTimestamped(device.m_dev, device.m_tracker, &rightVel, 3, &device.m_lastreport_time);
			osvrDeviceTrackerSendAccelerationTimestamped(device.m_dev, device.m_tracker, &rightAcc, 3, &device.m_lastreport_time);

			/*
			Report Buttons
			*/
			unsigned int leftButtons = data.left_Controller_Data.Buttons;
			unsigned int rightButtons = data.right_Controller_Data.Buttons;
			int leftTrigger = 0;
			int rightTrigger = 0;

			for (unsigned int bid = 0; bid < 5; ++bid) {
				OSVR_ButtonState leftValue = leftButtons & 1 << bid ? OSVR_BUTTON_PRESSED : OSVR_BUTTON_NOT_PRESSED;
				OSVR_ButtonState rightValue = rightButtons & 1 << bid ? OSVR_BUTTON_PRESSED : OSVR_BUTTON_NOT_PRESSED;
				if (bid == 1) {
					leftTrigger = (int)leftValue;
					rightTrigger = (int)rightValue;
				}
				osvrDeviceButtonSetValueTimestamped(device.m_dev, device.m_button, leftValue, bid, &device.m_lastreport_time);
				osvrDeviceButtonSetValueTimestamped(device.m_dev, device.m_button, rightValue, bid+6, &device.m_lastreport_time);
			}
			osvrDeviceAnalogSetValueTimestamped(device.m_dev, device.m_analog, leftTrigger, 2, &device.m_lastreport_time);
			osvrDeviceAnalogSetValueTimestamped(device.m_dev, device.m_analog, rightTrigger, 6, &device.m_lastreport_time);

			/*
			Report Analog Touchpad
			*/
			double leftx = data.left_Controller_Data.ControllerTouchAxis.x;
			double lefty = data.left_Controller_Data.ControllerTouchAxis.y;
			double rightx = data.right_Controller_Data.ControllerTouchAxis.x;
			double righty = data.right_Controller_Data.ControllerTouchAxis.y;
			osvrDeviceAnalogSetValueTimestamped(device.m_dev, device.m_analog, leftx, 0, &device.m_lastreport_time);
			osvrDeviceAnalogSetValueTimestamped(device.m_dev, device.m_analog, lefty, 1, &device.m_lastreport_time);
			osvrDeviceAnalogSetValueTimestamped(device.m_dev, device.m_analog, rightx, 4, &device.m_lastreport_time);
			osvrDeviceAnalogSetValueTimestamped(device.m_dev, device.m_analog, righty, 5, &device.m_lastreport_time);
			/*
			Report Status
			*/
			float leftBattery = (float)data.left_Controller_Data.ControllerBattery / 3.0f;
			float rightBattery = (float)data.right_Controller_Data.ControllerBattery / 3.0f;
			float baseBattery = (float)data.baseStationData.BaseStationPower / 3.0f;
			osvrDeviceAnalogSetValueTimestamped(device.m_dev, device.m_analog, leftBattery, 3, &device.m_lastreport_time);
			osvrDeviceAnalogSetValueTimestamped(device.m_dev, device.m_analog, rightBattery, 7, &device.m_lastreport_time);
			osvrDeviceAnalogSetValueTimestamped(device.m_dev, device.m_analog, baseBattery, 8, &device.m_lastreport_time);
		}

		void SetPose(OSVR_PoseState& pose, NOLO::Vector3& pos, NOLO::Quaternion& rot) {
			SetPosition(pose.translation, pos);
			SetRotation(pose.rotation, rot);
		}

		void SetVelocity(OSVR_VelocityState& vel, NOLO::Vector3& linear, NOLO::Vector3& angular) {
			SetLinearVelocity(vel, linear);
			SetAngularVelocity(vel, angular);
		}

		void SetAcceleration(OSVR_AccelerationState& acc, NOLO::Vector3& linear, NOLO::Vector3& angular) {
			SetLinearAcceleration(acc, linear);
			SetAngularAcceleration(acc, angular);
		}

		void SetPosition(OSVR_PositionState& pos, NOLO::Vector3& data) {
			double translationScale = 1.0f;
			osvrVec3SetX(&pos, translationScale * data.x);
			osvrVec3SetY(&pos, translationScale * data.y);
			osvrVec3SetZ(&pos, translationScale * -data.z);
		}

		void SetRotation(OSVR_Quaternion& quat, NOLO::Quaternion& data) {
			osvrQuatSetW(&quat, data.w);
			osvrQuatSetX(&quat, -data.x);
			osvrQuatSetY(&quat, -data.y);
			osvrQuatSetZ(&quat, data.z);
		}

		void SetLinearVelocity(OSVR_VelocityState& vel, NOLO::Vector3& data) {
			double velocityScale = 1.0f;
			osvrVec3SetX(&vel.linearVelocity, velocityScale * data.x);
			osvrVec3SetY(&vel.linearVelocity, velocityScale * data.y);
			osvrVec3SetZ(&vel.linearVelocity, velocityScale * data.z);
			vel.linearVelocityValid = true;
		}

		void SetAngularVelocity(OSVR_VelocityState& vel, NOLO::Vector3& data) {
			double angularVelocityScale = 1.0f;
			double angularVelocityDt = 1.0f;
			double i = data.x;
			double j = data.y;
			double k = data.z;
			double mag = sqrt(i*i + j*j + k*k);
			osvrQuatSetW(&vel.angularVelocity.incrementalRotation, angularVelocityScale * mag);
			osvrQuatSetX(&vel.angularVelocity.incrementalRotation, angularVelocityScale * i);
			osvrQuatSetY(&vel.angularVelocity.incrementalRotation, angularVelocityScale * k);
			osvrQuatSetZ(&vel.angularVelocity.incrementalRotation, angularVelocityScale * -j);
			vel.angularVelocity.dt = angularVelocityDt;
			vel.angularVelocityValid = true;
		}

		void SetLinearAcceleration(OSVR_AccelerationState& acc, NOLO::Vector3& data) {
			double accelerationScale = 1.0f;
			osvrVec3SetX(&acc.linearAcceleration, accelerationScale * data.x);
			osvrVec3SetY(&acc.linearAcceleration, accelerationScale * data.y);
			osvrVec3SetZ(&acc.linearAcceleration, accelerationScale * -data.z);
			acc.linearAccelerationValid = true;
		}

		void SetAngularAcceleration(OSVR_AccelerationState& acc, NOLO::Vector3& data) {
			double angularAccelerationScale = 1.0f;
			double angularAcceleartionDt = 1.0f;
			double i = data.x;
			double j = data.y;
			double k = data.z;
			double mag = sqrt(i*i + j*j + k*k);
			osvrQuatSetW(&acc.angularAcceleration.incrementalRotation, angularAccelerationScale * mag);
			osvrQuatSetX(&acc.angularAcceleration.incrementalRotation, angularAccelerationScale * i);
			osvrQuatSetY(&acc.angularAcceleration.incrementalRotation, angularAccelerationScale * k);
			osvrQuatSetZ(&acc.angularAcceleration.incrementalRotation, angularAccelerationScale * -j);
			acc.angularAcceleration.dt = angularAcceleartionDt;
			acc.angularAccelerationValid = true;
		}

		// Sets vibration strength in percent
		int setVibration(unsigned char left,
			unsigned char right) {
			int left_percentage = (int)(floor(double(left) / 255.0 * 100.0));
			int right_percentage = (int)(floor(double(right) / 255.0 * 100.0));
		}

	private:
		osvr::pluginkit::DeviceToken m_dev;
		bool m_is_nolo_connected;
		bool m_new_data_available;
		OSVR_AnalogDeviceInterface m_analog;
		OSVR_ButtonDeviceInterface m_button;
		OSVR_TrackerDeviceInterface m_tracker;
		OSVR_TimeValue m_lastreport_time;
		OSVR_Vec3 m_last_home;
		double m_last_axis[NUM_AXIS];
	};

	class HardwareDetection {
	public:
		HardwareDetection() : m_found(false) {}
		OSVR_ReturnCode operator()(OSVR_PluginRegContext ctx) {

			// TODO: probe for new devices only
			if (m_found)
				return OSVR_RETURN_SUCCESS;

			osvr::pluginkit::registerObjectForDeletion
				(ctx, new NoloDevice(ctx));
			m_found = true;

			return OSVR_RETURN_SUCCESS;
		}

	private:
		/// @brief Have we found our device yet? (this limits the plugin to one
		/// instance)
		bool m_found;
	};
} // namespace

OSVR_PLUGIN(com_osvr_Nolo) {
	osvr::pluginkit::PluginContext context(ctx);

	/// Register a detection callback function object.
	context.registerHardwareDetectCallback(new HardwareDetection());

	return OSVR_RETURN_SUCCESS;
}

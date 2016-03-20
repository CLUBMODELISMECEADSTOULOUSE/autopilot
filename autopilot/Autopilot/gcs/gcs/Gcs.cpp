/*
 * Gcs.cpp
 *
 *  Created on: 12 mai 2015
 *      Author: Aberzen
 */

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include "Gcs.hpp"
#include <system/system/System.hpp>
#include <hw/led/Led.hpp>

#include <Arduino.h>



mavlink_system_t mavlink_system = {
	    GCS_SYS_ID, // sysid: Used by the MAVLink message_xx_send() convenience function
	    GCS_COMP_ID, // compid: Used by the MAVLink message_xx_send() convenience function
	    MAV_TYPE_QUADROTOR, // type: Unused, can be used by user to store the system's type
	    MAV_STATE_UNINIT, // state: Unused, can be used by user to store the system's state
	    MAV_MODE_PREFLIGHT, // mode: Unused, can be used by user to store the system's mode
	    0 // nav_mode: Unused, can be used by user to store the system's navigation mode
};

namespace mavlink {

hw::Led myBlueLed(25);
hw::Led myRedLed(27);


Gcs::Gcs(mavlink_channel_t chan)
: _chan(chan),
  _deferredMessagesFlags(0),
  _deferredMessagesStart(0),
  _deferredMessagesEnd(0),
  _lastHeartbeatDate(millis()),
  _lastParamDate(millis()),
  _paramListRequested(false),
  _paramListCurrentIdx(0),
  _lastWpDate(millis()),
  _wpListRequested(false),
  _wpListCurrentIdx(0),
  _streamRateRawSensors(),
  _streamRateExtendedStatus(),
  _streamRateRCChannels(),
  _streamRatePosition(),
  _streamRateRawController(),
  _streamRateExtra1(),
  _streamRateExtra2(),
  _streamRateExtra3(),
  _flag_DEFERRED_MESSAGES_HEARTBEAT(false),
  _flag_DEFERRED_MESSAGES_NEXT_PARAM(false),
  _flag_DEFERRED_MESSAGES_NEXT_WP(false),
  _flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU1(false),
  _flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU2(false),
  _flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU3(false),
  _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_1(false),
  _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_2(false),
  _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT(false),
  _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW(false),
  _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT(false),
  _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT(false),
  _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS(false),
  _flag_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN(false),
  _flag_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT(false),
  _flag_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT(false),
  _flag_DEFERRED_MESSAGES_EXTRA1_ATTITUDE(false),
  _flag_DEFERRED_MESSAGES_EXTRA1_SIM_STATE(false),
  _flag_DEFERRED_MESSAGES_EXTRA2_VFR_HUD(false),
  _flag_DEFERRED_MESSAGES_EXTRA3_AHRS(false),
  _flag_DEFERRED_MESSAGES_EXTRA3_HWSTATUS(false)
{
	myBlueLed.switchOff();
	myRedLed.switchOff();

}

Gcs::~Gcs()
{
}

void Gcs::initialize()
{
	_streamRateRawSensors.start(50);
	_streamRatePosition.start(50);
	_streamRateRCChannels.start(50);
	_streamRateRawController.start(50);
	_streamRateExtendedStatus.start(50);
	_streamRateExtra1.start(50);
	_streamRateExtra2.start(50);
	_streamRateExtra3.start(50);
}

void Gcs::processNewMessages()
{
	mavlink_message_t* msg = NULL;

	/*Get the channel */
	mavlink::Channel* channel = mavlink::Channel::getChannel(_chan);

	if (channel != NULL)
	{
		/* Receive message */
		if (channel->receiveMessage(&msg))
		{
			handleMessage(msg);
		}
	}
}

void Gcs::processServices()
{
	// Process internal services in order to know
	// if messages are ready to be sent
	processServiceHeartbeat();
	processServiceParam();
	processServiceMission();
	processServiceStream();

	// Process messages that are ready to send
	sendDeferredMessages();
}

void Gcs::handleMessageRequestDataStream(uint8_t sysid, uint8_t compId, const mavlink_request_data_stream_t& packet)
{
	uint16_t count = 0;

	if (!checkTarget(packet.target_system, packet.target_component))
		return;

	if (packet.start_stop!=0)
	{
		switch(packet.req_stream_id)
		{

		case MAV_DATA_STREAM_ALL:
//			_streamRateRawSensors.start(packet.req_message_rate);
//			_streamRateExtendedStatus.start(packet.req_message_rate);
			_streamRateRCChannels.start(packet.req_message_rate);
//			_streamRateRawController.start(packet.req_message_rate);
//			_streamRatePosition.start(packet.req_message_rate);
//			_streamRateExtra1.start(packet.req_message_rate);
//			_streamRateExtra2.start(packet.req_message_rate);
//			_streamRateExtra3.start(packet.req_message_rate);
			break;

		case MAV_DATA_STREAM_RAW_SENSORS:
//			_streamRateRawSensors.start(packet.req_message_rate);
			break;
		case MAV_DATA_STREAM_EXTENDED_STATUS:
//			_streamRateExtendedStatus.start(packet.req_message_rate);
			break;
		case MAV_DATA_STREAM_RC_CHANNELS:
			_streamRateRCChannels.start(packet.req_message_rate);
			break;
		case MAV_DATA_STREAM_RAW_CONTROLLER:
//			_streamRateRawController.start(packet.req_message_rate);
			break;
		//case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
		//	streamRateRawSensorFusion.set_and_save(freq);
		//	break;
		case MAV_DATA_STREAM_POSITION:
//			_streamRatePosition.start(packet.req_message_rate);
			break;
		case MAV_DATA_STREAM_EXTRA1:
//			_streamRateExtra1.start(packet.req_message_rate);
			break;
		case MAV_DATA_STREAM_EXTRA2:
//			_streamRateExtra2.start(packet.req_message_rate);
			break;
		case MAV_DATA_STREAM_EXTRA3:
//			_streamRateExtra3.start(packet.req_message_rate);
			break;
		default:
			// unsupported
			break;
		}
	}
	else
	{
		switch(packet.req_stream_id)
		{

		case MAV_DATA_STREAM_ALL:
			_streamRateRawSensors.stop();
			_streamRateExtendedStatus.stop();
			_streamRateRCChannels.stop();
			_streamRateRawController.stop();
			_streamRatePosition.stop();
			_streamRateExtra1.stop();
			_streamRateExtra2.stop();
			_streamRateExtra3.stop();
			break;

		case MAV_DATA_STREAM_RAW_SENSORS:
			_streamRateRawSensors.stop();
			break;
		case MAV_DATA_STREAM_EXTENDED_STATUS:
			_streamRateExtendedStatus.stop();
			break;
		case MAV_DATA_STREAM_RC_CHANNELS:
			_streamRateRCChannels.stop();
			break;
		case MAV_DATA_STREAM_RAW_CONTROLLER:
			_streamRateRawController.stop();
			break;
		//case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
		//	streamRateRawSensorFusion.set_and_save(freq);
		//	break;
		case MAV_DATA_STREAM_POSITION:
			_streamRatePosition.stop();
			break;
		case MAV_DATA_STREAM_EXTRA1:
			_streamRateExtra1.stop();
			break;
		case MAV_DATA_STREAM_EXTRA2:
			_streamRateExtra2.stop();
			break;
		case MAV_DATA_STREAM_EXTRA3:
			_streamRateExtra3.stop();
			break;
		default:
			// unsupported
			break;
		}
	}
}
void Gcs::handleMessageCommandLong(uint8_t sysid, uint8_t compId, const mavlink_command_long_t& packet)
{
//	uint8_t result = MAV_RESULT_UNSUPPORTED;

	if (!checkTarget(packet.target_system, packet.target_component))
		return;

	/*

	switch(packet.command)
	{

	case MAV_CMD_NAV_LOITER_UNLIM:
		set_mode(LOITER);
		result = MAV_RESULT_ACCEPTED;
		break;

	case MAV_CMD_NAV_RETURN_TO_LAUNCH:
		set_mode(RTL);
		result = MAV_RESULT_ACCEPTED;
		break;

	case MAV_CMD_NAV_LAND:
		set_mode(LAND);
		result = MAV_RESULT_ACCEPTED;
		break;

	case MAV_CMD_MISSION_START:
		set_mode(AUTO);
		result = MAV_RESULT_ACCEPTED;
		break;

	case MAV_CMD_PREFLIGHT_CALIBRATION:
		if (packet.param1 == 1 ||
				packet.param2 == 1 ||
				packet.param3 == 1) {
			imu.init_accel(mavlink_delay, flash_leds);
		}
		if (packet.param4 == 1) {
			trim_radio();
		}
		result = MAV_RESULT_ACCEPTED;
		break;

	case MAV_CMD_COMPONENT_ARM_DISARM:
		if (packet.target_component == MAV_COMP_ID_SYSTEM_CONTROL) {
			if (packet.param1 == 1.0f) {
				init_arm_motors();
				result = MAV_RESULT_ACCEPTED;
			} else if (packet.param1 == 0.0f)  {
				init_disarm_motors();
				result = MAV_RESULT_ACCEPTED;
			} else {
				result = MAV_RESULT_UNSUPPORTED;
			}
		} else {
			result = MAV_RESULT_UNSUPPORTED;
		}
		break;
	default:
		result = MAV_RESULT_UNSUPPORTED;
		break;
	}
	*/

//	mavlink_msg_command_ack_send(
//			chan,
//			packet.command,
//			result);
}

void Gcs::handleMessageSetMode(uint8_t sysid, uint8_t compId, const mavlink_set_mode_t& packet)
{
	/*
	if (!(packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) {
		// we ignore base_mode as there is no sane way to map
		// from that bitmap to a APM flight mode. We rely on
		// custom_mode instead.
		break;
	}
	switch (packet.custom_mode)
	{
	case STABILIZE:
	case ACRO:
	case ALT_HOLD:
	case AUTO:
	case GUIDED:
	case LOITER:
	case RTL:
	case CIRCLE:
	case POSITION:
	case LAND:
	case OF_LOITER:
		set_mode(packet.custom_mode);
		break;
	}
	*/
}

void Gcs::handleMessageMissionRequestList(uint8_t sysid, uint8_t compId, const mavlink_mission_request_list_t& packet)
{
	if (!checkTarget(packet.target_system, packet.target_component))
		return;
}

void Gcs::handleMessageMissionRequest(uint8_t sysid, uint8_t compId, const mavlink_mission_request_t& packet)
{
	if (!checkTarget(packet.target_system, packet.target_component))
		return;
}

void Gcs::handleMessageMissionAck(uint8_t sysid, uint8_t compId, const mavlink_mission_ack_t& packet)
{
	if (!checkTarget(packet.target_system,packet.target_component))
		return;
}

void Gcs::handleMessageParamRequestList(uint8_t sysid, uint8_t compId, const mavlink_param_request_list_t& packet)
{
	if (!checkTarget(packet.target_system,packet.target_component))
		return;

	_lastParamDate = millis() + 3000;
	_paramListCurrentIdx = 0;
	_paramListRequested = true;
	myBlueLed.switchOn();
	myRedLed.switchOff();
}

void Gcs::handleMessageParamRequestRead(uint8_t sysid, uint8_t compId, const mavlink_param_request_read_t& packet)
{
	if (!checkTarget(packet.target_system,packet.target_component))
		return;

	ParameterMgt::Param param;
	ParameterMgt& mgt = system::system.getParameterMgt();
	float param_value = 0;

	bool res = mgt.find(packet.param_id,param.idx);
	uint16_t count = mgt.getParamCount();

	if (res && packet.param_index == param.idx)
	{
		mgt.getInfo(param.idx,&param.type, &param.name[0]);
		// Read parameter information
		mgt.read(param.idx,param.value);
		switch(param.type)
		{
		case MAVLINK_TYPE_INT8_T:
			param_value = (float) param.value.INT8;
			break;
		case MAVLINK_TYPE_UINT8_T:
			param_value = (float) param.value.UINT8;
			break;
		case MAVLINK_TYPE_INT16_T:
			param_value = (float) param.value.INT16;
			break;
		case MAVLINK_TYPE_UINT16_T:
			param_value = (float) param.value.UINT16;
			break;
		case MAVLINK_TYPE_INT32_T:
			param_value = (float) param.value.INT32;
			break;
		case MAVLINK_TYPE_UINT32_T:
			param_value = (float) param.value.UINT32;
			break;
		case MAVLINK_TYPE_FLOAT:
			param_value = (float) param.value.REAL32;
			break;
		}

		// Send parameter
		mavlink_msg_param_value_send(
				_chan,
				param.name,
				param_value,
				param.type,
				mgt.getParamCount(),
				param.idx);
	}
}

void Gcs::handleMessageMissionClearAll(uint8_t sysid, uint8_t compId, const mavlink_mission_clear_all_t& packet)
{
	if (!checkTarget(packet.target_system, packet.target_component))
		return;
}

void Gcs::handleMessageMissionSetCurrent(uint8_t sysid, uint8_t compId, const mavlink_mission_set_current_t& packet)
{
	if (!checkTarget(packet.target_system,packet.target_component))
		return;
}


void Gcs::handleMessageMissionCount(uint8_t sysid, uint8_t compId, const mavlink_mission_count_t& packet)
{
	if (!checkTarget(packet.target_system,packet.target_component))
		return;
}

void Gcs::handleMessageSetMagOffsets(uint8_t sysid, uint8_t compId, const mavlink_set_mag_offsets_t& packet)
{
	if (!checkTarget(packet.target_system,packet.target_component))
		return;
}

void Gcs::handleMessageMissionItem(uint8_t sysid, uint8_t compId, const mavlink_mission_item_t& packet)
{
	if (!checkTarget(packet.target_system,packet.target_component))
		return;
}


void Gcs::handleMessageParamSet(uint8_t sysid, uint8_t compId, const mavlink_param_set_t& packet)
{
	myBlueLed.switchOn();
	myRedLed.switchOff();
	if (!checkTarget(packet.target_system, packet.target_component))
	{
		myRedLed.switchOn();
		return;
	}

	ParameterMgt::Param param;
	ParameterMgt& mgt = system::system.getParameterMgt();
	float param_value = 0;

	bool res = mgt.find(packet.param_id,param.idx);


	if (res)
	{
		mgt.getInfo(param.idx,&param.type, &param.name[0]);
		param.value.REAL32 = packet.param_value;
		mgt.write(param.idx,param.value);
//		switch (param.type)
//		{
//		case MAV_PARAM_TYPE_INT8:
//			param.value.INT8 = (int8_t) packet.param_value;
//			mgt.write(param.idx,param.value);
//			break;
//		case MAV_PARAM_TYPE_UINT8:
//			param.value.UINT8 = (uint8_t) packet.param_value;
//			mgt.write(param.idx,param.value);
//			break;
//		case MAV_PARAM_TYPE_INT16:
//			param.value.INT16 = (int16_t) packet.param_value;
//			mgt.write(param.idx,param.value);
//			break;
//		case MAV_PARAM_TYPE_UINT16:
//			param.value.UINT16 = (uint16_t) packet.param_value;
//			mgt.write(param.idx,param.value);
//			break;
//		case MAV_PARAM_TYPE_INT32:
//			param.value.INT32 = (int32_t) packet.param_value;
//			mgt.write(param.idx,param.value);
//			break;
//		case MAV_PARAM_TYPE_UINT32:
//			param.value.UINT32 = (uint32_t) packet.param_value;
//			mgt.write(param.idx,param.value);
//			break;
//		case MAV_PARAM_TYPE_REAL32:
//			param.value.REAL32 = packet.param_value;
//			mgt.write(param.idx,param.value);
//			break;
//		default:
//			break;
//		}

		// Read parameter information
		mgt.read(param.idx,param.value);
		switch(param.type)
		{
		case MAVLINK_TYPE_INT8_T:
			param_value = (float) param.value.INT8;
			break;
		case MAVLINK_TYPE_UINT8_T:
			param_value = (float) param.value.UINT8;
			break;
		case MAVLINK_TYPE_INT16_T:
			param_value = (float) param.value.INT16;
			break;
		case MAVLINK_TYPE_UINT16_T:
			param_value = (float) param.value.UINT16;
			break;
		case MAVLINK_TYPE_INT32_T:
			param_value = (float) param.value.INT32;
			break;
		case MAVLINK_TYPE_UINT32_T:
			param_value = (float) param.value.UINT32;
			break;
		case MAVLINK_TYPE_FLOAT:
			param_value = (float) param.value.REAL32;
			break;
		}

		// Send parameter
		mavlink_msg_param_value_send(
				_chan,
				param.name,
				param_value,
				param.type,
				mgt.getParamCount(),
				param.idx);
	}
	else
	{
		myBlueLed.switchOff();
		myRedLed.switchOn();
		return;
	}
	myBlueLed.switchOff();
	myRedLed.switchOff();
}

void Gcs::handleMessageChannelsOverride(uint8_t sysid, uint8_t compId, const mavlink_rc_channels_override_t& packet)
{
	if (!checkTarget(packet.target_system,packet.target_component))
		return;
}

void Gcs::handleMessageHilState(uint8_t sysid, uint8_t compId, const mavlink_hil_state_t& packet)
{
}

void Gcs::handleMessageHeartbeat(uint8_t sysid, uint8_t compId, const mavlink_heartbeat_t& packet)
{
}

void Gcs::handleMessageGpsRawInt(uint8_t sysid, uint8_t compId, const mavlink_gps_raw_int_t& packet)
{
}

void Gcs::handleMessageRawImu(uint8_t sysid, uint8_t compId, const mavlink_raw_imu_t& packet)
{
}

void Gcs::handleMessageRawPressure(uint8_t sysid, uint8_t compId, const mavlink_raw_pressure_t& packet)
{
}

void Gcs::handleMessageDigicamConfigure(uint8_t sysid, uint8_t compId, const mavlink_digicam_configure_t& packet)
{
}

void Gcs::handleMessageDigicamControl(uint8_t sysid, uint8_t compId, const mavlink_digicam_control_t& packet)
{
}

void Gcs::handleMessageMountConfigure(uint8_t sysid, uint8_t compId, const mavlink_mount_configure_t& packet)
{
}

void Gcs::handleMessageMountControl(uint8_t sysid, uint8_t compId, const mavlink_mount_control_t& packet)
{
}

void Gcs::handleMessageMountStatus(uint8_t sysid, uint8_t compId, const mavlink_mount_status_t& packet)
{

}

void Gcs::handleMessageRadio(uint8_t sysid, uint8_t compId, const mavlink_radio_t& packet)
{
}

void Gcs::handleMessageFencePoint(uint8_t sysid, uint8_t compId, const mavlink_fence_point_t& packet)
{
}

void Gcs::handleMessageFenceFetchPoint(uint8_t sysid, uint8_t compId, const mavlink_fence_fetch_point_t& packet)
{
	if (!checkTarget(packet.target_system, packet.target_component))
		return;
}

void Gcs::handleMessage(const mavlink_message_t* msg)
{
	switch (msg->msgid)
	{
	case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:     //66
	{
		// decode
		mavlink_request_data_stream_t packet;
		mavlink_msg_request_data_stream_decode(msg, &packet);
		// process
		handleMessageRequestDataStream(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_COMMAND_LONG:
	{
		// decode
		mavlink_command_long_t packet;
		mavlink_msg_command_long_decode(msg, &packet);
		// process
		handleMessageCommandLong(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_SET_MODE:      //11
	{
		// decode
		mavlink_set_mode_t packet;
		mavlink_msg_set_mode_decode(msg, &packet);
		// process
		handleMessageSetMode(msg->sysid, msg->compid, packet);
		break;
	}
/*	case MAVLINK_MSG_ID_SET_NAV_MODE:
	{
		// decode
		mavlink_set_nav_mode_t packet;
		mavlink_msg_set_nav_mode_decode(msg, &packet);
		// process
		handleMessageSetNavMode(msg->sysid, msg->compid, packet);
		break;
	} */
	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:     //43
	{
		// decode
		mavlink_mission_request_list_t packet;
		mavlink_msg_mission_request_list_decode(msg, &packet);
		// process
		handleMessageMissionRequestList(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_REQUEST:     // 40
	{
		// decode
		mavlink_mission_request_t packet;
		mavlink_msg_mission_request_decode(msg, &packet);
		// process
		handleMessageMissionRequest(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_ACK:     //47
	{
		// decode
		mavlink_mission_ack_t packet;
		mavlink_msg_mission_ack_decode(msg, &packet);
		// process
		handleMessageMissionAck(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:     // 21
	{
		// decode
		mavlink_param_request_list_t packet;
		mavlink_msg_param_request_list_decode(msg, &packet);
		// process
		handleMessageParamRequestList(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
	{
		// decode
		mavlink_param_request_read_t packet;
		mavlink_msg_param_request_read_decode(msg, &packet);
		// process
		handleMessageParamRequestRead(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:     // 45
	{
		// decode
		mavlink_mission_clear_all_t packet;
		mavlink_msg_mission_clear_all_decode(msg, &packet);
		// process
		handleMessageMissionClearAll(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_SET_CURRENT:     // 41
	{
		// decode
		mavlink_mission_set_current_t packet;
		mavlink_msg_mission_set_current_decode(msg, &packet);
		// process
		handleMessageMissionSetCurrent(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_MISSION_COUNT:     // 44
	{
		// decode
		mavlink_mission_count_t packet;
		mavlink_msg_mission_count_decode(msg, &packet);
		// process
		handleMessageMissionCount(msg->sysid, msg->compid, packet);
		break;
	}
#ifdef MAVLINK_MSG_ID_SET_MAG_OFFSETS
	case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
	{
		// decode
		mavlink_set_mag_offsets_t packet;
		mavlink_msg_set_mag_offsets_decode(msg, &packet);
		// process
		handleMessageSetMagOffsets(msg->sysid, msg->compid, packet);
		break;
	}
#endif
	case MAVLINK_MSG_ID_MISSION_ITEM:     //39
	{
		// decode
		mavlink_mission_item_t packet;
		mavlink_msg_mission_item_decode(msg, &packet);
		// process
		handleMessageMissionItem(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_PARAM_SET:     // 23
	{
		// decode
		mavlink_param_set_t packet;
		mavlink_msg_param_set_decode(msg, &packet);
		// process
		handleMessageParamSet(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: //70
	{
		// decode
		mavlink_rc_channels_override_t packet;
		mavlink_msg_rc_channels_override_decode(msg, &packet);
		// process
		handleMessageChannelsOverride(msg->sysid, msg->compid, packet);
		break;
	}
#if HIL_MODE != HIL_MODE_DISABLED
	case MAVLINK_MSG_ID_HIL_STATE:
	{
		// decode
		mavlink_hil_state_t packet;
		mavlink_msg_hil_state_decode(msg, &packet);
		// process
		handleMessageHilState(msg->sysid, msg->compid, packet);
		break;
	}
#endif //  HIL_MODE != HIL_MODE_DISABLED
	case MAVLINK_MSG_ID_HEARTBEAT:
	{
		// decode
		mavlink_heartbeat_t packet;
		mavlink_msg_heartbeat_decode(msg, &packet);
		// process
		handleMessageHeartbeat(msg->sysid, msg->compid, packet);
		break;
	}
   // This is used both as a sensor and to pass the location
   // in HIL_ATTITUDE mode.
   case MAVLINK_MSG_ID_GPS_RAW_INT:
   {
	   // decode
	   mavlink_gps_raw_int_t packet;
	   mavlink_msg_gps_raw_int_decode(msg, &packet);
	   // process
	   handleMessageGpsRawInt(msg->sysid, msg->compid, packet);
	   break;
   }
	case MAVLINK_MSG_ID_RAW_IMU: // 28
	{
		// decode
		mavlink_raw_imu_t packet;
		mavlink_msg_raw_imu_decode(msg, &packet);
		// process
		handleMessageRawImu(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_RAW_PRESSURE: //29
	{
		// decode
		mavlink_raw_pressure_t packet;
		mavlink_msg_raw_pressure_decode(msg, &packet);
		// process
		handleMessageRawPressure(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_DIGICAM_CONFIGURE:
	{
		// decode
		mavlink_digicam_configure_t packet;
		mavlink_msg_digicam_configure_decode(msg, &packet);
		// process
		handleMessageDigicamConfigure(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_DIGICAM_CONTROL:
	{
		// decode
		mavlink_digicam_control_t packet;
		mavlink_msg_digicam_control_decode(msg, &packet);
		// process
		handleMessageDigicamControl(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_MOUNT_CONFIGURE:
	{
		// decode
		mavlink_mount_configure_t packet;
		mavlink_msg_mount_configure_decode(msg, &packet);
		// process
		handleMessageMountConfigure(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_MOUNT_CONTROL:
	{
		// decode
		mavlink_mount_control_t packet;
		mavlink_msg_mount_control_decode(msg, &packet);
		// process
		handleMessageMountControl(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_MOUNT_STATUS:
	{
		// decode
		mavlink_mount_status_t packet;
		mavlink_msg_mount_status_decode(msg, &packet);
		// process
		handleMessageMountStatus(msg->sysid, msg->compid, packet);
		break;
	}
	case MAVLINK_MSG_ID_RADIO:
	{
		// decode
		mavlink_radio_t packet;
		mavlink_msg_radio_decode(msg, &packet);
		// process
		handleMessageRadio(msg->sysid, msg->compid, packet);
		break;
	}
	// receive an AP_Limits fence point from GCS and store in EEPROM
	// receive a fence point from GCS and store in EEPROM
	case MAVLINK_MSG_ID_FENCE_POINT:
	{
		// decode
		mavlink_fence_point_t packet;
		mavlink_msg_fence_point_decode(msg, &packet);
		// process
		handleMessageFencePoint(msg->sysid, msg->compid, packet);
		break;
	}
	// send a fence point to GCS
	case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
	{
		// decode
		mavlink_fence_fetch_point_t packet;
		mavlink_msg_fence_fetch_point_decode(msg, &packet);
		// process
		handleMessageFenceFetchPoint(msg->sysid, msg->compid, packet);
		break;
	}
	}
}

void Gcs::processServiceHeartbeat()
{
	uint16_t now = millis();
	if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_HEARTBEAT))
	{
		uint16_t dt = now - _lastHeartbeatDate;
		if (dt>=GCS_PERIOD_HEARTBEAT_MS)
		{
			pushDeferredMessage(E_DEFERRED_MESSAGES_HEARTBEAT);
		}
	}
}

void Gcs::processServiceParam()
{
	if (_paramListRequested)
	{
//		hw::Serial& com = system::getCom0();
//		char buffer[100];
//		sprintf(buffer,"_deferredMessagesFlags=%ud\n",_deferredMessagesFlags);
//		com.write((uint8_t *)buffer, strlen(buffer));

		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_NEXT_PARAM))
		{
			uint16_t now = millis();
			uint16_t dt = now - _lastParamDate;
//			sprintf(buffer,"dt=%d\n",dt);
//			com.write((uint8_t *)buffer, strlen(buffer));
			if (dt>=GCS_PERIOD_PARAM_MS)
			{
				pushDeferredMessage(E_DEFERRED_MESSAGES_NEXT_PARAM);
			}
		}
	}
}

void Gcs::processServiceMission()
{
	uint16_t now = millis();
	if (_wpListRequested)
	{
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_NEXT_WP))
		{
			uint16_t dt = now - _lastParamDate;
			if (dt>GCS_PERIOD_MISSION_MS)
			{
				pushDeferredMessage(E_DEFERRED_MESSAGES_NEXT_WP);
			}
		}
	}
}

void Gcs::processServiceStream()
{
	// Check stream
	if (_streamRateRawSensors.eval())
	{
		_streamRateRawSensors.restart();
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_RAW_SENSORS_IMU1))
			pushDeferredMessage(E_DEFERRED_MESSAGES_RAW_SENSORS_IMU1);
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_RAW_SENSORS_IMU2))
			pushDeferredMessage(E_DEFERRED_MESSAGES_RAW_SENSORS_IMU2);
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_RAW_SENSORS_IMU3))
			pushDeferredMessage(E_DEFERRED_MESSAGES_RAW_SENSORS_IMU3);
	}

	if (_streamRateExtendedStatus.eval())
	{
		_streamRateExtendedStatus.restart();
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTENDED_STATUS_1))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTENDED_STATUS_1);
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTENDED_STATUS_2))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTENDED_STATUS_2);
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT);
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS);
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW);
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT);
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT);
	}

	if (_streamRateRCChannels.eval())
	{
		_streamRateRCChannels.restart();
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT))
			pushDeferredMessage(E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT);
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN))
			pushDeferredMessage(E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN);
	}

	if (_streamRateRawController.eval())
	{
		_streamRateRawController.restart();
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT))
			pushDeferredMessage(E_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT);
	}

	if (_streamRatePosition.eval())
	{
		_streamRatePosition.restart();
	}

	if (_streamRateExtra1.eval())
	{
		_streamRateExtra1.restart();
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTRA1_ATTITUDE))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTRA1_ATTITUDE);
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTRA1_SIM_STATE))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTRA1_SIM_STATE);
	}

	if (_streamRateExtra2.eval())
	{
		_streamRateExtra2.restart();
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTRA2_VFR_HUD))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTRA2_VFR_HUD);
	}

	if (_streamRateExtra3.eval())
	{
		_streamRateExtra3.restart();
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTRA3_AHRS))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTRA3_AHRS);
		if ( ! isLockedDeferred(E_DEFERRED_MESSAGES_EXTRA3_HWSTATUS))
			pushDeferredMessage(E_DEFERRED_MESSAGES_EXTRA3_HWSTATUS);
	}

}

void Gcs::sendDeferredMessages()
{
	E_DEFERRED_MESSAGES next;
	bool result = true;
	while(readDeferredMessage(next))
	{
		switch(next)
		{
		case E_DEFERRED_MESSAGES_HEARTBEAT:
			result = sendHeartbeat();
			break;

		case E_DEFERRED_MESSAGES_NEXT_PARAM:
			result = sendNextParam();
			break;

		case E_DEFERRED_MESSAGES_NEXT_WP:
			result = sendNextWaypoint();
			break;

		case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU1:
			result = sendImu1();
			break;

		case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU2:
			result = sendImu2();
			break;

		case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU3:
			result = sendImu3();
			break;

		case E_DEFERRED_MESSAGES_EXTENDED_STATUS_1:
			result = sendStatus1();
			break;

		case E_DEFERRED_MESSAGES_EXTENDED_STATUS_2:
			result = sendStatus2();
			break;

		case E_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT:
			result = sendCurrentWaypoint();
			break;

		case E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW:
			result = sendStatusGpsRaw();
			break;

		case E_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT:
			result = sendStatusNavControllerOutput();
			break;

		case E_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT:
			result = sendStatusLimit();
			break;

		case E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS:
			result = sendStatusGps();
			break;

		case E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN:
			result = sendRadioIn();
			break;

		case E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT:
			result = sendRadioOut();
			break;

		case E_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT:
			result = sendServoOut();
			break;

		case E_DEFERRED_MESSAGES_EXTRA1_ATTITUDE:
			result = sendExtraAttitude();
			break;

		case E_DEFERRED_MESSAGES_EXTRA1_SIM_STATE:
			result = sendExtraSimState();
			break;

		case E_DEFERRED_MESSAGES_EXTRA2_VFR_HUD:
			result = sendExtraVfrHud();
			break;

		case E_DEFERRED_MESSAGES_EXTRA3_AHRS:
			result = sendExtraAhrs();
			break;

		case E_DEFERRED_MESSAGES_EXTRA3_HWSTATUS:
			result = sendExtraHwStatus();
			break;

		default:
			result = true; // set to true in case the ID is unknown ==> equivalent to drop
			break;
		}

		if (result)
		{
			clearDeferredMessage();
		}
		else
		{
			// No more space into the buffer
			// Leave the loop
			break;
		}
	}
}
bool Gcs::sendHeartbeat()
{
	bool result = false;
	Channel* channel = Channel::getChannel(_chan);

	if (channel != NULL)
	{
		if (channel->canSend(MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_HEARTBEAT_LEN))
		{
			mavlink_msg_heartbeat_send(
					_chan,
					mavlink_system.type,
					MAV_AUTOPILOT_GENERIC,
					mavlink_system.mode,
					mavlink_system.nav_mode,
					mavlink_system.state);

			result = true;
			_lastHeartbeatDate = millis();
		}
	}
	return result;
}

bool Gcs::sendNextParam()
{
	bool result = false;
	mavlink::ParameterMgt& mgt = system::system.getParameterMgt();
	uint16_t count = mgt.getParamCount();
	Channel* channel = Channel::getChannel(_chan);

	if (channel != NULL)
	{
		if (channel->canSend(MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_PARAM_VALUE_LEN))
		{
			// Read parameter information
			enum MAV_PARAM_TYPE type;
			char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN+1];
			result = mgt.getInfo(
					_paramListCurrentIdx,
					&type,
					&name[0]);

			if (result)
			{
				// If exists, then, get values
				mavlink::ParameterMgt::Value value;
				mgt.read(_paramListCurrentIdx,value);
				float param_value = 0;
				switch(type)
				{
				case MAVLINK_TYPE_INT8_T:
					param_value = (float) value.INT8;
					break;
				case MAVLINK_TYPE_UINT8_T:
					param_value = (float) value.UINT8;
					break;
				case MAVLINK_TYPE_INT16_T:
					param_value = (float) value.INT16;
					break;
				case MAVLINK_TYPE_UINT16_T:
					param_value = (float) value.UINT16;
					break;
				case MAVLINK_TYPE_INT32_T:
					param_value = (float) value.INT32;
					break;
				case MAVLINK_TYPE_UINT32_T:
					param_value = (float) value.UINT32;
					break;
				case MAVLINK_TYPE_FLOAT:
					param_value = (float) value.REAL32;
					break;
				}
				param_value = value.REAL32;
//				hw::Serial& com = system::getCom0();
//				char buffer[100];
//				sprintf(buffer,"sending %d/%d\n",_paramListCurrentIdx,count);
//				com.write((uint8_t *)buffer, strlen(buffer));
//				sprintf(buffer,"\tnamed %s\n",name);
//				com.write((uint8_t *)buffer, strlen(buffer));
//				sprintf(buffer,"\tvalue %d\n",value.INT32);
//				com.write((uint8_t *)buffer, strlen(buffer));
//				sprintf(buffer,"\ttype %d\n",type);
//				com.write((uint8_t *)buffer, strlen(buffer));
				// Send parameter
				mavlink_msg_param_value_send(
						_chan,
						name,
						param_value,
						type,
						count,
						_paramListCurrentIdx);

				myRedLed.blink();
				result = true;
				_paramListCurrentIdx++;
				_lastParamDate= millis();
				if (_paramListCurrentIdx >= count)
				{
					// Terminated
					_paramListCurrentIdx = 0;
					_paramListRequested = false;
					myBlueLed.switchOff();
					myRedLed.switchOff();
				}
			}
			else
			{
				// error, stop there
				_paramListCurrentIdx = 0;
				_paramListRequested = false;
				myBlueLed.switchOff();
				myRedLed.switchOff();
			}
		}
	}
	return result;
}

bool Gcs::sendNextWaypoint()
{
	return true;
}

bool Gcs::sendImu1()
{
	bool result = false;
	Channel* channel = Channel::getChannel(_chan);

	if (channel->canSend(MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_RAW_IMU_LEN))
	{

		uint64_t now = micros();
		mavlink_msg_raw_imu_send(
			_chan,
			now,
			system::system.dataPool.imuAccRaw_U.x,
			system::system.dataPool.imuAccRaw_U.y,
			system::system.dataPool.imuAccRaw_U.z,
			system::system.dataPool.imuRateRaw_U.x,
			system::system.dataPool.imuRateRaw_U.y,
			system::system.dataPool.imuRateRaw_U.z,
			system::system.dataPool.compassMagRaw_U.x,
			system::system.dataPool.compassMagRaw_U.y,
			system::system.dataPool.compassMagRaw_U.z);

		result = true;
	}
	return result;
}

bool Gcs::sendImu2()
{
	bool result = false;
	Channel* channel = Channel::getChannel(_chan);
	if (channel->canSend(MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_RAW_PRESSURE_LEN))
	{
		uint64_t now = micros();
		mavlink_msg_raw_pressure_send(
			_chan,
			now,
			system::system.dataPool.baroPressRaw,
			0,
			0,
			system::system.dataPool.baroTempRaw);

		result = true;
	}
	return result;
}

bool Gcs::sendImu3()
{
	bool result = false;
	Channel* channel = Channel::getChannel(_chan);
	if (channel->canSend(MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_ATTITUDE_LEN))
	{
		uint32_t now = millis();
		float roll,
			  pitch,
			  yaw;
		system::system.dataPool.estDcm_IB.to_euler(&roll, &pitch, &yaw);
		mavlink_msg_attitude_send(
			_chan,
			now,
			roll,
			pitch,
			yaw,
			system::system.dataPool.estRate_B.x,
			system::system.dataPool.estRate_B.y,
			system::system.dataPool.estRate_B.z);

		result = true;
	}
	return result;
}

bool Gcs::sendStatus1()
{
	return true;
}

bool Gcs::sendStatus2()
{
	return true;
}

bool Gcs::sendCurrentWaypoint()
{
	return true;
}

bool Gcs::sendStatusGpsRaw()
{
	return true;
}

bool Gcs::sendStatusNavControllerOutput()
{
	return true;
}

bool Gcs::sendStatusLimit()
{
	return true;
}

bool Gcs::sendStatusGps()
{
	return true;
}

bool Gcs::sendRadioIn()
{
	bool result = false;
	Channel* channel = Channel::getChannel(_chan);
	if (channel->canSend(MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_RADIO_LEN))
//	if (channel->canSend(MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN))
	{
		uint32_t now = millis();
		mavlink_msg_rc_channels_raw_send(
			_chan,
			now,
			0,
			system::system.dataPool.pwm_inputs[0],
			system::system.dataPool.pwm_inputs[1],
			system::system.dataPool.pwm_inputs[2],
			system::system.dataPool.pwm_inputs[3],
			system::system.dataPool.pwm_inputs[4],
			system::system.dataPool.pwm_inputs[5],
			system::system.dataPool.pwm_inputs[6],
			system::system.dataPool.pwm_inputs[7],
			0);

//		mavlink_msg_rc_channels_scaled_send(
//			_chan,
//			now,
//			0,
//			system::system.dataPool.pwm_inputs[0],
//			system::system.dataPool.pwm_inputs[1],
//			system::system.dataPool.pwm_inputs[2],
//			system::system.dataPool.pwm_inputs[3],
//			system::system.dataPool.pwm_inputs[4],
//			system::system.dataPool.pwm_inputs[5],
//			system::system.dataPool.pwm_inputs[6],
//			system::system.dataPool.pwm_inputs[7],
//			0);

		result = true;
	}
	return result;
}

bool Gcs::sendRadioOut()
{
	bool result = false;
	Channel* channel = Channel::getChannel(_chan);
	if (channel->canSend(MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_RADIO_LEN))
	{
		uint32_t now = millis();
		result = true;
	}
	return result;
}

bool Gcs::sendServoOut()
{
	bool result = false;
	Channel* channel = Channel::getChannel(_chan);
	if (channel->canSend(MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN))
	{
		uint32_t now = millis();
		mavlink_msg_servo_output_raw_send(
				_chan,
				now,
				0,
				system::system.dataPool.pwm_outputs[0],
				system::system.dataPool.pwm_outputs[1],
				system::system.dataPool.pwm_outputs[2],
				system::system.dataPool.pwm_outputs[3],
				system::system.dataPool.pwm_outputs[4],
				system::system.dataPool.pwm_outputs[5],
				system::system.dataPool.pwm_outputs[6],
				system::system.dataPool.pwm_outputs[7]);
		result = true;
	}
	return result;
}

bool Gcs::sendExtraAttitude()
{
	return true;
}

bool Gcs::sendExtraSimState()
{
	return true;
}

bool Gcs::sendExtraVfrHud()
{
	return true;
}

bool Gcs::sendExtraAhrs()
{
	return true;
}

bool Gcs::sendExtraHwStatus()
{
	return true;
}


void Gcs::pushDeferredMessage(const E_DEFERRED_MESSAGES& msg)
{
	if (_deferredMessagesEnd - _deferredMessagesStart < E_DEFERRED_MESSAGES_NB)
	{
		lockDeferred(msg);
		_deferredMessages[_deferredMessagesEnd++] = msg;
		if (_deferredMessagesEnd >= (E_DEFERRED_MESSAGES_NB+1))
			_deferredMessagesEnd = 0;
	}
}

bool Gcs::readDeferredMessage(E_DEFERRED_MESSAGES& msg) const
{
	bool result = false;
	if (_deferredMessagesStart != _deferredMessagesEnd)
	{
		msg = _deferredMessages[_deferredMessagesStart];
		result = true;
	}
	return result;
}

void Gcs::clearDeferredMessage()
{
	if (_deferredMessagesStart != _deferredMessagesEnd)
	{
		unlockDeferred(_deferredMessages[_deferredMessagesStart++]);
		if (_deferredMessagesStart>=(E_DEFERRED_MESSAGES_NB+1))
			_deferredMessagesStart = 0;
	}
}

bool Gcs::checkTarget(uint8_t target_system, uint8_t target_component)
{
	return
			   (mavlink_system.sysid == target_system)
			&& (mavlink_system.compid == target_component || target_component == MAV_COMP_ID_ALL);
}

void Gcs::lockDeferred(const E_DEFERRED_MESSAGES& deferredMessageId)
{
	switch (deferredMessageId)
	{
	case E_DEFERRED_MESSAGES_HEARTBEAT:
		_flag_DEFERRED_MESSAGES_HEARTBEAT = true;
		break;
	case E_DEFERRED_MESSAGES_NEXT_PARAM:
		_flag_DEFERRED_MESSAGES_NEXT_PARAM = true;
		break;
	case E_DEFERRED_MESSAGES_NEXT_WP:
		_flag_DEFERRED_MESSAGES_NEXT_WP = true;
		break;
	case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU1:
		_flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU1 = true;
		break;
	case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU2:
		_flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU2 = true;
		break;
	case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU3:
		_flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU3 = true;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_1:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_1 = true;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_2:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_2 = true;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT = true;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW = true;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT = true;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT = true;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS = true;
		break;
	case E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN:
		_flag_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN = true;
		break;
	case E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT:
		_flag_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT = true;
		break;
	case E_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT:
		_flag_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT = true;
		break;
	case E_DEFERRED_MESSAGES_EXTRA1_ATTITUDE:
		_flag_DEFERRED_MESSAGES_EXTRA1_ATTITUDE = true;
		break;
	case E_DEFERRED_MESSAGES_EXTRA1_SIM_STATE:
		_flag_DEFERRED_MESSAGES_EXTRA1_SIM_STATE = true;
		break;
	case E_DEFERRED_MESSAGES_EXTRA2_VFR_HUD:
		_flag_DEFERRED_MESSAGES_EXTRA2_VFR_HUD = true;
		break;
	case E_DEFERRED_MESSAGES_EXTRA3_AHRS:
		_flag_DEFERRED_MESSAGES_EXTRA3_AHRS = true;
		break;
	case E_DEFERRED_MESSAGES_EXTRA3_HWSTATUS:
		_flag_DEFERRED_MESSAGES_EXTRA3_HWSTATUS = true;
		break;
	}
}

bool Gcs::isLockedDeferred(const E_DEFERRED_MESSAGES& deferredMessageId)
{
	bool result = false;;
	switch (deferredMessageId)
	{
	case E_DEFERRED_MESSAGES_HEARTBEAT:
		result = (_flag_DEFERRED_MESSAGES_HEARTBEAT == true);
		break;
	case E_DEFERRED_MESSAGES_NEXT_PARAM:
		result = (_flag_DEFERRED_MESSAGES_NEXT_PARAM == true);
		break;
	case E_DEFERRED_MESSAGES_NEXT_WP:
		result = (_flag_DEFERRED_MESSAGES_NEXT_WP == true);
		break;
	case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU1:
		result = (_flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU1 == true);
		break;
	case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU2:
		result = (_flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU2 == true);
		break;
	case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU3:
		result = (_flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU3 == true);
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_1:
		result = (_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_1 == true);
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_2:
		result = (_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_2 == true);
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT:
		result = (_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT == true);
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW:
		result = (_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW == true);
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT:
		result = (_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT == true);
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT:
		result = (_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT == true);
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS:
		result = (_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS == true);
		break;
	case E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN:
		result = (_flag_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN == true);
		break;
	case E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT:
		result = (_flag_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT == true);
		break;
	case E_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT:
		result = (_flag_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT == true);
		break;
	case E_DEFERRED_MESSAGES_EXTRA1_ATTITUDE:
		result = (_flag_DEFERRED_MESSAGES_EXTRA1_ATTITUDE == true);
		break;
	case E_DEFERRED_MESSAGES_EXTRA1_SIM_STATE:
		result = (_flag_DEFERRED_MESSAGES_EXTRA1_SIM_STATE == true);
		break;
	case E_DEFERRED_MESSAGES_EXTRA2_VFR_HUD:
		result = (_flag_DEFERRED_MESSAGES_EXTRA2_VFR_HUD == true);
		break;
	case E_DEFERRED_MESSAGES_EXTRA3_AHRS:
		result = (_flag_DEFERRED_MESSAGES_EXTRA3_AHRS == true);
		break;
	case E_DEFERRED_MESSAGES_EXTRA3_HWSTATUS:
		result = (_flag_DEFERRED_MESSAGES_EXTRA3_HWSTATUS == true);
		break;
	}
	return result;
}

void Gcs::unlockDeferred(const E_DEFERRED_MESSAGES& deferredMessageId)
{
	switch (deferredMessageId)
	{
	case E_DEFERRED_MESSAGES_HEARTBEAT:
		_flag_DEFERRED_MESSAGES_HEARTBEAT = false;
		break;
	case E_DEFERRED_MESSAGES_NEXT_PARAM:
		_flag_DEFERRED_MESSAGES_NEXT_PARAM = false;
		break;
	case E_DEFERRED_MESSAGES_NEXT_WP:
		_flag_DEFERRED_MESSAGES_NEXT_WP = false;
		break;
	case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU1:
		_flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU1 = false;
		break;
	case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU2:
		_flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU2 = false;
		break;
	case E_DEFERRED_MESSAGES_RAW_SENSORS_IMU3:
		_flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU3 = false;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_1:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_1 = false;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_2:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_2 = false;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT = false;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW = false;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT = false;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT = false;
		break;
	case E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS:
		_flag_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS = false;
		break;
	case E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN:
		_flag_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN = false;
		break;
	case E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT:
		_flag_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT = false;
		break;
	case E_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT:
		_flag_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT = false;
		break;
	case E_DEFERRED_MESSAGES_EXTRA1_ATTITUDE:
		_flag_DEFERRED_MESSAGES_EXTRA1_ATTITUDE = false;
		break;
	case E_DEFERRED_MESSAGES_EXTRA1_SIM_STATE:
		_flag_DEFERRED_MESSAGES_EXTRA1_SIM_STATE = false;
		break;
	case E_DEFERRED_MESSAGES_EXTRA2_VFR_HUD:
		_flag_DEFERRED_MESSAGES_EXTRA2_VFR_HUD = false;
		break;
	case E_DEFERRED_MESSAGES_EXTRA3_AHRS:
		_flag_DEFERRED_MESSAGES_EXTRA3_AHRS = false;
		break;
	case E_DEFERRED_MESSAGES_EXTRA3_HWSTATUS:
		_flag_DEFERRED_MESSAGES_EXTRA3_HWSTATUS = false;
		break;
	}
}

} /* namespace mavlink */

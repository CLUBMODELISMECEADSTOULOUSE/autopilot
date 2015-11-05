/*
 * Gcs.hpp
 *
 *  Created on: 12 mai 2015
 *      Author: Aberzen
 */

#ifndef GCS_HPP_
#define GCS_HPP_

#include <stdint.h>
#include <gcs/channel/Channel.hpp>
#include <infra/timer/Timer.hpp>
#include <gcs/mission/Mission.hpp>

#ifndef GCS_PERIOD_MS
#define GCS_PERIOD_MS (10)
#endif

#ifndef GCS_PERIOD_HEARTBEAT_MS
#define GCS_PERIOD_HEARTBEAT_MS (1000)
#endif

#ifndef GCS_PERIOD_PARAM_MS
#define GCS_PERIOD_PARAM_MS (100)
#endif

#ifndef GCS_PERIOD_MISSION_MS
#define GCS_PERIOD_MISSION_MS (100)
#endif

#ifndef GCS_SYSTEM_ID
#define GCS_SYSTEM_ID (MAV_AUTOPILOT_GENERIC_MISSION_FULL)
#endif

#ifndef GCS_COMPONENT_ID
#define GCS_COMPONENT_ID (MAVLINK_COM)
#endif

#ifndef GCS_SYS_ID
#define GCS_SYS_ID (7)
#endif

#ifndef GCS_COMP_ID
#define GCS_COMP_ID (1)
#endif


//extern mavlink_system_t mavlink_system;

namespace mavlink {

class Gcs {
protected:
	typedef enum {
		E_DEFERRED_MESSAGES_HEARTBEAT = 0,
		E_DEFERRED_MESSAGES_NEXT_PARAM,
		E_DEFERRED_MESSAGES_NEXT_WP,

		E_DEFERRED_MESSAGES_RAW_SENSORS_IMU1,
		E_DEFERRED_MESSAGES_RAW_SENSORS_IMU2,
		E_DEFERRED_MESSAGES_RAW_SENSORS_IMU3,

		E_DEFERRED_MESSAGES_EXTENDED_STATUS_1,
		E_DEFERRED_MESSAGES_EXTENDED_STATUS_2,
		E_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT,
		E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW,
		E_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT,
		E_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT,
		E_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS,

		E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN,
		E_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT,

		E_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT,

		E_DEFERRED_MESSAGES_EXTRA1_ATTITUDE,
		E_DEFERRED_MESSAGES_EXTRA1_SIM_STATE,

		E_DEFERRED_MESSAGES_EXTRA2_VFR_HUD,

		E_DEFERRED_MESSAGES_EXTRA3_AHRS,
		E_DEFERRED_MESSAGES_EXTRA3_HWSTATUS,

		E_DEFERRED_MESSAGES_NB
	} E_DEFERRED_MESSAGES;

public:
	Gcs(mavlink_channel_t chan);
	virtual ~Gcs();
	void initialize();
	void processNewMessages();
	void processServices();

protected:
	bool checkTarget(uint8_t target_system, uint8_t target_component);

	void processServiceHeartbeat();
	void processServiceParam();
	void processServiceMission();
	void processServiceStream();

	void handleMessage(const mavlink_message_t* msg);

	void handleMessageRequestDataStream(uint8_t sysid, uint8_t compId, const mavlink_request_data_stream_t& packet);
	void handleMessageCommandLong(uint8_t sysid, uint8_t compId, const mavlink_command_long_t& packet);
	void handleMessageSetMode(uint8_t sysid, uint8_t compId, const mavlink_set_mode_t& packet);
	void handleMessageMissionRequestList(uint8_t sysid, uint8_t compId, const mavlink_mission_request_list_t& packet);
	void handleMessageMissionRequest(uint8_t sysid, uint8_t compId, const mavlink_mission_request_t& packet);
	void handleMessageMissionAck(uint8_t sysid, uint8_t compId, const mavlink_mission_ack_t& packet);
	void handleMessageParamRequestList(uint8_t sysid, uint8_t compId, const mavlink_param_request_list_t& packet);
	void handleMessageParamRequestRead(uint8_t sysid, uint8_t compId, const mavlink_param_request_read_t& packet);
	void handleMessageMissionClearAll(uint8_t sysid, uint8_t compId, const mavlink_mission_clear_all_t& packet);
	void handleMessageMissionSetCurrent(uint8_t sysid, uint8_t compId, const mavlink_mission_set_current_t& packet);
	void handleMessageMissionCount(uint8_t sysid, uint8_t compId, const mavlink_mission_count_t& packet);
	void handleMessageSetMagOffsets(uint8_t sysid, uint8_t compId, const mavlink_set_mag_offsets_t& packet);
	void handleMessageMissionItem(uint8_t sysid, uint8_t compId, const mavlink_mission_item_t& packet);
	void handleMessageParamSet(uint8_t sysid, uint8_t compId, const mavlink_param_set_t& packet);
	void handleMessageChannelsOverride(uint8_t sysid, uint8_t compId, const mavlink_rc_channels_override_t& packet);
	void handleMessageHilState(uint8_t sysid, uint8_t compId, const mavlink_hil_state_t& packet);
	void handleMessageHeartbeat(uint8_t sysid, uint8_t compId, const mavlink_heartbeat_t& packet);
	void handleMessageGpsRawInt(uint8_t sysid, uint8_t compId, const mavlink_gps_raw_int_t& packet);
	void handleMessageRawImu(uint8_t sysid, uint8_t compId, const mavlink_raw_imu_t& packet);
	void handleMessageRawPressure(uint8_t sysid, uint8_t compId, const mavlink_raw_pressure_t& packet);
	void handleMessageDigicamConfigure(uint8_t sysid, uint8_t compId, const mavlink_digicam_configure_t& packet);
	void handleMessageDigicamControl(uint8_t sysid, uint8_t compId, const mavlink_digicam_control_t& packet);
	void handleMessageMountConfigure(uint8_t sysid, uint8_t compId, const mavlink_mount_configure_t& packet);
	void handleMessageMountControl(uint8_t sysid, uint8_t compId, const mavlink_mount_control_t& packet);
	void handleMessageMountStatus(uint8_t sysid, uint8_t compId, const mavlink_mount_status_t& packet);
	void handleMessageRadio(uint8_t sysid, uint8_t compId, const mavlink_radio_t& packet);
	void handleMessageFencePoint(uint8_t sysid, uint8_t compId, const mavlink_fence_point_t& packet);
	void handleMessageFenceFetchPoint(uint8_t sysid, uint8_t compId, const mavlink_fence_fetch_point_t& packet);


	void sendDeferredMessages();
	bool sendHeartbeat();
	bool sendNextParam();
	bool sendNextWaypoint();
	bool sendImu1();
	bool sendImu2();
	bool sendImu3();
	bool sendStatus1();
	bool sendStatus2();
	bool sendCurrentWaypoint();
	bool sendStatusGpsRaw();
	bool sendStatusNavControllerOutput();
	bool sendStatusLimit();
	bool sendStatusGps();
	bool sendRadioIn();
	bool sendRadioOut();
	bool sendServoOut();
	bool sendExtraAttitude();
	bool sendExtraSimState();
	bool sendExtraVfrHud();
	bool sendExtraAhrs();
	bool sendExtraHwStatus();


	void pushDeferredMessage(const E_DEFERRED_MESSAGES& msg) ;
	bool readDeferredMessage(E_DEFERRED_MESSAGES& msg) const ;
	void clearDeferredMessage() ;
	void lockDeferred(const E_DEFERRED_MESSAGES& deferredMessageId);
	void unlockDeferred(const E_DEFERRED_MESSAGES& deferredMessageId);
	bool isLockedDeferred(const E_DEFERRED_MESSAGES& deferredMessageId);


	Mission _mission;

	mavlink_channel_t _chan;


	uint32_t _deferredMessagesFlags;
	E_DEFERRED_MESSAGES _deferredMessages[E_DEFERRED_MESSAGES_NB+1];
	uint8_t _deferredMessagesStart;
	uint8_t _deferredMessagesEnd;

	uint16_t _lastHeartbeatDate;

	uint16_t _lastParamDate;
	bool _paramListRequested;
	uint8_t _paramListCurrentIdx;

	uint16_t _lastWpDate;
	bool _wpListRequested;
	uint16_t _wpListCurrentIdx;

	infra::Timer _streamRateRawSensors;
	infra::Timer _streamRateExtendedStatus;
	infra::Timer _streamRateRCChannels;
	infra::Timer _streamRatePosition;
	infra::Timer _streamRateRawController;
	infra::Timer _streamRateExtra1;
	infra::Timer _streamRateExtra2;
	infra::Timer _streamRateExtra3;


	bool _flag_DEFERRED_MESSAGES_HEARTBEAT;
	bool _flag_DEFERRED_MESSAGES_NEXT_PARAM;
	bool _flag_DEFERRED_MESSAGES_NEXT_WP;
	bool _flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU1;
	bool _flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU2;
	bool _flag_DEFERRED_MESSAGES_RAW_SENSORS_IMU3;
	bool _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_1;
	bool _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_2;
	bool _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_CURRENT_WAYPOINT;
	bool _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS_RAW;
	bool _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_NAV_CONTROLLER_OUTPUT;
	bool _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_LIMIT;
	bool _flag_DEFERRED_MESSAGES_EXTENDED_STATUS_GPS;
	bool _flag_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_IN;
	bool _flag_DEFERRED_MESSAGES_RC_CHANNEL_RADIO_OUT;
	bool _flag_DEFERRED_MESSAGES_RAW_CONTROLLER_SERVO_OUT;
	bool _flag_DEFERRED_MESSAGES_EXTRA1_ATTITUDE;
	bool _flag_DEFERRED_MESSAGES_EXTRA1_SIM_STATE;
	bool _flag_DEFERRED_MESSAGES_EXTRA2_VFR_HUD;
	bool _flag_DEFERRED_MESSAGES_EXTRA3_AHRS;
	bool _flag_DEFERRED_MESSAGES_EXTRA3_HWSTATUS;

};

} /* namespace mavlink */
#endif /* GCS_HPP_ */

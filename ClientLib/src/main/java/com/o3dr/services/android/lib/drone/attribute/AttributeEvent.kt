package com.o3dr.services.android.lib.drone.attribute

/**
 * Stores all possible drone events.
 */
object AttributeEvent {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.attribute.event"

    /**
     * Attitude attribute events.
     */
    const val ATTITUDE_UPDATED = "$PACKAGE_NAME.ATTITUDE_UPDATED"

    /**
     * Signals an autopilot error.
     *
     * @see {@link com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra.EXTRA_AUTOPILOT_ERROR_ID}
     */
    const val AUTOPILOT_ERROR = "$PACKAGE_NAME.AUTOPILOT_ERROR"

    /**
     * Event describing a message received from the autopilot.
     * The message content can be retrieved using the [com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra.EXTRA_AUTOPILOT_MESSAGE] key.
     * The message level can be retrieved using the [com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra.EXTRA_AUTOPILOT_MESSAGE_LEVEL] key.
     */
    const val AUTOPILOT_MESSAGE = "$PACKAGE_NAME.AUTOPILOT_MESSAGE"

    /**
     * Event to signal cancellation of the magnetometer calibration process.
     */
    const val CALIBRATION_MAG_CANCELLED = "$PACKAGE_NAME.CALIBRATION_MAG_CANCELLED"

    /**
     * Signals completion of the magnetometer calibration.
     *
     * @see {@link AttributeEventExtra.EXTRA_CALIBRATION_MAG_RESULT}
     */
    const val CALIBRATION_MAG_COMPLETED = "$PACKAGE_NAME.CALIBRATION_MAG_COMPLETED"

    /**
     * Provides progress updates for the magnetometer calibration.
     *
     * @see {@link AttributeEventExtra.EXTRA_CALIBRATION_MAG_PROGRESS}
     */
    const val CALIBRATION_MAG_PROGRESS = "$PACKAGE_NAME.CALIBRATION_MAG_PROGRESS"
    const val CALIBRATION_IMU = "$PACKAGE_NAME.CALIBRATION_IMU"
    const val CALIBRATION_IMU_TIMEOUT = "$PACKAGE_NAME.CALIBRATION_IMU_TIMEOUT"
    const val FOLLOW_START = "$PACKAGE_NAME.FOLLOW_START"
    const val FOLLOW_STOP = "$PACKAGE_NAME.FOLLOW_STOP"
    const val FOLLOW_UPDATE = "$PACKAGE_NAME.FOLLOW_UPDATE"

    /**
     * Camera attribute events.
     */
    const val CAMERA_UPDATED = "$PACKAGE_NAME.CAMERA_UPDATED"
    const val CAMERA_FOOTPRINTS_UPDATED = "$PACKAGE_NAME.CAMERA_FOOTPRINTS_UPDATED"

    /** AirCommander event when onboard */
    const val AIRCOMMANDER_HEARTBEAT = "$PACKAGE_NAME.AIRCOMMANDER_HEARTBEAT"

    /**
     * GuidedState attribute events.
     */
    const val GUIDED_POINT_UPDATED = "$PACKAGE_NAME.GUIDED_POINT_UPDATED"

    /**
     * Mission attribute events.
     */
    const val MISSION_UPDATED = "$PACKAGE_NAME.MISSION_UPDATED"
    const val MISSION_DRONIE_CREATED = "$PACKAGE_NAME.MISSION_DRONIE_CREATED"
    const val MISSION_SENT = "$PACKAGE_NAME.MISSION_SENT"
    const val MISSION_RECEIVED = "$PACKAGE_NAME.MISSION_RECEIVED"
    const val MISSION_ITEM_UPDATED = "$PACKAGE_NAME.MISSION_ITEM_UPDATED"
    const val MISSION_ITEM_REACHED = "$PACKAGE_NAME.MISSION_ITEM_REACHED"
    /*
     * Parameter attribute events.
     */
    /**
     * Event to signal the start of parameters refresh from the vehicle.
     *
     * @see {@link com.o3dr.services.android.lib.drone.property.Parameters}
     *
     * @see {@link com.o3dr.services.android.lib.drone.property.Parameter}
     */
    const val PARAMETERS_REFRESH_STARTED = "$PACKAGE_NAME.PARAMETERS_REFRESH_STARTED"

    /**
     * Event to signal the completion of the parameters refresh from the vehicle.
     *
     * @see {@link com.o3dr.services.android.lib.drone.property.Parameters}
     *
     * @see {@link com.o3dr.services.android.lib.drone.property.Parameter}
     */
    const val PARAMETERS_REFRESH_COMPLETED = "$PACKAGE_NAME.PARAMETERS_REFRESH_ENDED"

    /**
     * Event to signal receipt of a single parameter from the vehicle. During a parameters refresh, this event will
     * fire as many times as the count of the set of parameters being refreshed.
     * Allows listeners to keep track of the parameters refresh progress.
     *
     * @see {@link AttributeEventExtra.EXTRA_PARAMETER_INDEX}
     *
     * @see {@link AttributeEventExtra.EXTRA_PARAMETERS_COUNT}
     *
     * @see {@link AttributeEventExtra.EXTRA_PARAMETER_NAME}
     *
     * @see {@link AttributeEventExtra.EXTRA_PARAMETER_VALUE}
     */
    const val PARAMETER_RECEIVED = "$PACKAGE_NAME.PARAMETERS_RECEIVED"

    /**
     * Event to signal update of the vehicle type.
     */
    const val TYPE_UPDATED = "$PACKAGE_NAME.TYPE_UPDATED"

    /**
     * Signal attribute events.
     */
    const val SIGNAL_UPDATED = "$PACKAGE_NAME.SIGNAL_UPDATED"
    const val SIGNAL_WEAK = "$PACKAGE_NAME.SIGNAL_WEAK"

    /**
     * Speed attribute events.
     */
    const val SPEED_UPDATED = "$PACKAGE_NAME.SPEED_UPDATED"

    /**
     * Battery attribute events.
     */
    const val BATTERY_UPDATED = "$PACKAGE_NAME.BATTERY_UPDATED"
    /*
     * State attribute events.
     */
    /**
     * Signals changes in the vehicle readiness (i.e: standby or active/airborne).
     */
    const val STATE_UPDATED = "$PACKAGE_NAME.STATE_UPDATED"

    /**
     * Signals changes in the vehicle arming state.
     */
    const val STATE_ARMING = "$PACKAGE_NAME.STATE_ARMING"
    const val STATE_CONNECTING = "$PACKAGE_NAME.STATE_CONNECTING"
    const val STATE_CONNECTED = "$PACKAGE_NAME.STATE_CONNECTED"
    const val STATE_DISCONNECTED = "$PACKAGE_NAME.STATE_DISCONNECTED"

    /**
     * Signals updates of the ekf status.
     * @see {@link com.o3dr.services.android.lib.drone.property.State}
     */
    const val STATE_EKF_REPORT = "$PACKAGE_NAME.STATE_EKF_REPORT"

    /**
     * Signals updates to the ekf position state.
     * @see {@link com.o3dr.services.android.lib.drone.property.State}
     */
    const val STATE_EKF_POSITION = "$PACKAGE_NAME.STATE_EKF_POSITION"

    /**
     * Signals update of the vehicle mode.
     * @see {@link com.o3dr.services.android.lib.drone.property.State}
     */
    const val STATE_VEHICLE_MODE = "$PACKAGE_NAME.STATE_VEHICLE_MODE"

    /**
     * Signals vehicle vibration updates.
     * @see {@link com.o3dr.services.android.lib.drone.property.State}
     */
    const val STATE_VEHICLE_VIBRATION = "$PACKAGE_NAME.STATE_VEHICLE_VIBRATION"

    /**
     * Signals vehicle UID updates.
     * @see {@link com.o3dr.services.android.lib.drone.property.State}
     */
    const val STATE_VEHICLE_UID = "$PACKAGE_NAME.STATE_VEHICLE_UID"

    /**
     * Home attribute events.
     */
    const val HOME_UPDATED = "$PACKAGE_NAME.HOME_UPDATED"

    /**
     * Gps' attribute events.
     */
    const val GPS_POSITION = "$PACKAGE_NAME.GPS_POSITION"
    const val GPS_FIX = "$PACKAGE_NAME.GPS_FIX"
    const val GPS2_FIX = "$PACKAGE_NAME.GPS2_FIX"
    const val GPS_COUNT = "$PACKAGE_NAME.GPS_COUNT"
    const val WARNING_NO_GPS = "$PACKAGE_NAME.WARNING_NO_GPS"
    const val HEARTBEAT_FIRST = "$PACKAGE_NAME.HEARTBEAT_FIRST"
    const val HEARTBEAT_RESTORED = "$PACKAGE_NAME.HEARTBEAT_RESTORED"
    const val HEARTBEAT_TIMEOUT = "$PACKAGE_NAME.HEARTBEAT_TIMEOUT"

    /**
     * Altitude's attribute events.
     */
    const val ALTITUDE_UPDATED = "$PACKAGE_NAME.ALTITUDE_UPDATED"

    /**
     * Signals the gimbal orientation was updated.
     *
     * @see {@link AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_PITCH}
     *
     * @see {@link AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_ROLL}
     *
     * @see {@link AttributeEventExtra.EXTRA_GIMBAL_ORIENTATION_YAW}
     */
    const val GIMBAL_ORIENTATION_UPDATED = "$PACKAGE_NAME.GIMBAL_ORIENTATION_UPDATED"

    /**
     * Signals an update to the return to me state.
     * Retrieves the current state via [AttributeEventExtra.EXTRA_RETURN_TO_ME_STATE]
     */
    const val RETURN_TO_ME_STATE_UPDATE = "$PACKAGE_NAME.RETURN_TO_ME_STATE_UPDATE"

    /**
     * Signals an update to fence status.
     */
    const val FENCE_STATUS = "$PACKAGE_NAME.FENCE_STATUS"

    /**
     * Signals an update to the autopilot version.
     */
    const val AUTOPILOT_VERSION = "$PACKAGE_NAME.AUTOPILOT_VERSION"
    const val ADSB_VEHICLE = "$PACKAGE_NAME.ADSB_VEHICLE"
    const val RANGE_FINDER = "$PACKAGE_NAME.RANGE_FINDER"
}

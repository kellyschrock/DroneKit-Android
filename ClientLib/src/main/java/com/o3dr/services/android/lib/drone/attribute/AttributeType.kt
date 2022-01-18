package com.o3dr.services.android.lib.drone.attribute

import com.o3dr.services.android.lib.drone.attribute.AttributeType

/**
 * Stores the set of attribute types.
 */
object AttributeType {
    private const val PACKAGE_NAME = "com.o3dr.services.android.lib.attribute"

    /**
     * Used to access the vehicle's altitude state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Altitude}
     */
    const val ALTITUDE = "$PACKAGE_NAME.ALTITUDE"

    /**
     * Used to access the vehicle's attitude state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Attitude}
     */
    const val ATTITUDE = "$PACKAGE_NAME.ATTITUDE"

    /**
     * Used to access the vehicle's battery state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Battery}
     */
    const val BATTERY = "$PACKAGE_NAME.BATTERY"
    const val BATTERY2 = "$PACKAGE_NAME.BATTERY2"

    /**
     * Used to access the set of camera information available for the connected drone.
     * @see {@link com.o3dr.services.android.lib.drone.property.CameraProxy}
     */
    const val CAMERA = "$PACKAGE_NAME.CAMERA"

    /**
     * Used to acces the vehicle's follow state.
     * @see {@link com.o3dr.services.android.lib.gcs.follow.FollowState}
     */
    const val FOLLOW_STATE = "$PACKAGE_NAME.FOLLOW_STATE"

    /**
     * Used to access the vehicle's guided state.
     * @see {@link com.o3dr.services.android.lib.drone.property.GuidedState}
     */
    const val GUIDED_STATE = "$PACKAGE_NAME.GUIDED_STATE"

    /**
     * Used to access the vehicle's gps state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Gps} object.
     */
    const val GPS = "$PACKAGE_NAME.GPS"

    /**
     * Used to access the vehicle's GPS2 state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Gps} object.
     */
    const val GPS2_RAW = "$PACKAGE_NAME.GPS2_RAW"

    /**
     * Used to access the vehicle's home state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Home}
     */
    const val HOME = "$PACKAGE_NAME.HOME"

    /**
     * Used to access the vehicle's mission state.
     * @see {@link com.o3dr.services.android.lib.drone.mission.Mission}
     */
    const val MISSION = "$PACKAGE_NAME.MISSION"

    /**
     * Used to access the vehicle's parameters.
     * @see {@link com.o3dr.services.android.lib.drone.property.Parameters}
     *
     * @see {@link com.o3dr.services.android.lib.drone.property.Parameter}
     */
    const val PARAMETERS = "$PACKAGE_NAME.PARAMETERS"

    /**
     * Used to access the vehicle's signal state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Signal}
     */
    const val SIGNAL = "$PACKAGE_NAME.SIGNAL"

    /**
     * Used to access the vehicle's speed info.
     * @see {@link com.o3dr.services.android.lib.drone.property.Speed}
     */
    const val SPEED = "$PACKAGE_NAME.SPEED"

    /**
     * Used to access the vehicle state.
     * @see {@link com.o3dr.services.android.lib.drone.property.State} object.
     */
    const val STATE = "$PACKAGE_NAME.STATE"

    /**
     * Used to access mavlink connection stats.
     */
    const val MAVLINK_STATS = "$PACKAGE_NAME.MAVLINK_STATS"

    /**
     * Used to access the vehicle type.
     * @see {@link com.o3dr.services.android.lib.drone.property.Type}
     */
    const val TYPE = "$PACKAGE_NAME.TYPE"

    /**
     * Used to retrieve the status of the currently or last running magnetometer calibration.
     * @see {@link com.o3dr.services.android.lib.drone.calibration.magnetometer.MagnetometerCalibrationStatus}
     */
    const val MAGNETOMETER_CALIBRATION_STATUS = "$PACKAGE_NAME.MAGNETOMETER_CALIBRATION_STATUS"

    /**
     * Used to retrieve the 'return to me' state.
     * @see {@link com.o3dr.services.android.lib.gcs.returnToMe.ReturnToMeState}
     */
    const val RETURN_TO_ME_STATE = "$PACKAGE_NAME.RETURN_TO_ME_STATE"

    /** Used to retrieve the fence status.
     * @see {@link com.o3dr.services.android.lib.drone.property.FenceStatus}
     */
    const val FENCE_STATUS = "$PACKAGE_NAME.FENCE_STATUS"

    /**
     * Used to retrieve Autopilot version
     * * @see [com.o3dr.services.android.lib.drone.property.AutopilotVersion]
     */
    const val AUTOPILOT_VERSION = "$PACKAGE_NAME.AUTOPILOT_VERSION"
    const val RANGE_FINDER = "$PACKAGE_NAME.RANGE_FINDER"
}

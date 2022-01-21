package org.droidplanner.services.android.impl.core.mission.waypoints

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import org.droidplanner.services.android.impl.core.mission.waypoints.SpatialCoordItem
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemType

/**
 * Handle spline waypoint mavlink packet generation.
 */
class SplineWaypointImpl : SpatialCoordItem {
    /**
     * Hold time in decimal seconds. (ignored by fixed wing, time to stay at
     * MISSION for rotary wing)
     */
    var delay = 0.0

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(mission: Mission?, coord: LatLongAlt?) : super(mission, coord) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission, null) {
        unpackMAVMessage(msg)
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT
        mavMsg.param1 = delay.toFloat()
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        super.unpackMAVMessage(mavMsg)
        delay = mavMsg.param1.toDouble()
    }

    override fun getType(): MissionItemType = MissionItemType.SPLINE_WAYPOINT
}

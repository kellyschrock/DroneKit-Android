package org.droidplanner.services.android.impl.core.mission.waypoints

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import org.droidplanner.services.android.impl.core.mission.waypoints.SpatialCoordItem
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemType

class WaypointImpl : SpatialCoordItem {
    var delay = 0.0
    var acceptanceRadius = 0.0
    var yawAngle = 0.0
    var orbitalRadius = 0.0
    var isOrbitCCW = false

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(mission: Mission?, coord: LatLongAlt?) : super(mission, coord) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission, null) {
        unpackMAVMessage(msg)
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.command = MAV_CMD.MAV_CMD_NAV_WAYPOINT
        mavMsg.param1 = delay.toFloat()
        mavMsg.param2 = acceptanceRadius.toFloat()
        mavMsg.param3 = (if (isOrbitCCW) orbitalRadius * -1.0 else orbitalRadius).toFloat()
        mavMsg.param4 = yawAngle.toFloat()
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        super.unpackMAVMessage(mavMsg)
        delay = mavMsg.param1.toDouble()
        acceptanceRadius = mavMsg.param2.toDouble()
        isOrbitCCW = mavMsg.param3 < 0
        orbitalRadius = Math.abs(mavMsg.param3).toDouble()
        yawAngle = mavMsg.param4.toDouble()
    }

    override fun getType(): MissionItemType = MissionItemType.WAYPOINT
}

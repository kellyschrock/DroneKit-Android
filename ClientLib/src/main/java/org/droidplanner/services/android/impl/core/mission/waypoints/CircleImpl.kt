package org.droidplanner.services.android.impl.core.mission.waypoints

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import com.MAVLink.enums.MAV_FRAME
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType
import java.util.*

class CircleImpl : SpatialCoordItem {
    var radius = 10.0
    var numberOfTurns = 1
        private set

    constructor(item: MissionItemImpl?) : super(item) {}
    constructor(mission: Mission?, coord: LatLongAlt?) : super(mission, coord) {}
    constructor(msg: msg_mission_item, mission: Mission?) : super(mission, null) {
        unpackMAVMessage(msg)
    }

    fun setTurns(turns: Int) {
        numberOfTurns = Math.abs(turns)
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list: MutableList<msg_mission_item> = ArrayList()
        packSingleCircle(list)
        return list
    }

    private fun packSingleCircle(list: MutableList<msg_mission_item>) {
        val mavMsg = msg_mission_item()
        list.add(mavMsg)
        mavMsg.autocontinue = 1
        mavMsg.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT.toShort()
        mavMsg.x = coordinate.latitude.toFloat()
        mavMsg.y = coordinate.longitude.toFloat()
        mavMsg.z = coordinate.altitude.toFloat()
        mavMsg.command = MAV_CMD.MAV_CMD_NAV_LOITER_TURNS
        mavMsg.param1 = Math.abs(numberOfTurns).toFloat()
        mavMsg.param3 = radius.toFloat()
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        super.unpackMAVMessage(mavMsg)
        setTurns(mavMsg.param1.toInt())
        radius = mavMsg.param3.toDouble()
    }

    override fun getType(): MissionItemType = MissionItemType.CIRCLE
}

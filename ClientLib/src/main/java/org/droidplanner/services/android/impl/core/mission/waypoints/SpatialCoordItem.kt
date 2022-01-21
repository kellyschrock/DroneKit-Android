package org.droidplanner.services.android.impl.core.mission.waypoints

import com.MAVLink.common.msg_mission_item
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl

abstract class SpatialCoordItem : MissionItemImpl {
    var coordinate: LatLongAlt = LatLongAlt(0.0, 0.0, 0.0)

    constructor(mission: Mission?, coord: LatLongAlt?) : super(mission) {
        coord?.let {
            coordinate = it
        }
    }

    constructor(item: MissionItemImpl?) : super(item) {
        coordinate = if (item is SpatialCoordItem) {
            item.coordinate
        } else {
            LatLongAlt(0.0, 0.0, 0.0)
        }
    }

    override fun packMissionItem(): List<msg_mission_item> {
        val list = super.packMissionItem()
        val mavMsg = list[0]
        mavMsg.x = coordinate.latitude.toFloat()
        mavMsg.y = coordinate.longitude.toFloat()
        mavMsg.z = coordinate.altitude.toFloat()
        return list
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {
        coordinate = LatLongAlt(mavMsg.x.toDouble(), mavMsg.y.toDouble(), mavMsg.z.toDouble())
    }

    fun setAltitude(altitude: Double) {
        coordinate.altitude = altitude
    }

    fun setPosition(position: LatLong?) {
        coordinate.set(position!!)
    }
}

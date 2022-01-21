package org.droidplanner.services.android.impl.core.mission

import android.util.Pair
import com.MAVLink.common.msg_mission_ack
import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import com.MAVLink.enums.MAV_FRAME
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.attribute.AttributeType
import com.o3dr.services.android.lib.drone.property.Attitude
import com.o3dr.services.android.lib.drone.property.Gps
import com.o3dr.services.android.lib.drone.property.Home
import com.o3dr.services.android.lib.drone.property.Parameter
import org.droidplanner.services.android.impl.core.drone.DroneInterfaces.DroneEventsType
import org.droidplanner.services.android.impl.core.drone.DroneVariable
import org.droidplanner.services.android.impl.core.drone.autopilot.apm.APMConstants
import org.droidplanner.services.android.impl.core.drone.autopilot.generic.GenericMavLinkDrone
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools
import org.droidplanner.services.android.impl.core.mission.commands.*
import org.droidplanner.services.android.impl.core.mission.waypoints.*
import java.util.*

/**
 * This implements a mavlink mission. A mavlink mission is a set of
 * commands/mission items to be carried out by the drone.
 */
class Mission(myDrone: GenericMavLinkDrone?) : DroneVariable<GenericMavLinkDrone?>(myDrone) {
    /**
     * Stores the set of mission items belonging to this mission.
     */
    private val items: MutableList<MissionItemImpl?> = ArrayList()
    private val componentItems: MutableList<MissionItemImpl?> = ArrayList()

    /**
     * Removes a waypoint from the mission's set of mission items.
     *
     * @param item waypoint to remove
     */
    fun removeWaypoint(item: MissionItemImpl?) {
        items.remove(item)
        notifyMissionUpdate()
    }

    /**
     * Removes a list of waypoints from the mission's set of mission items.
     *
     * @param toRemove list of waypoints to remove
     */
    fun removeWaypoints(toRemove: List<MissionItemImpl?>?) {
        items.removeAll(toRemove!!)
        notifyMissionUpdate()
    }

    /**
     * Add a list of waypoints to the mission's set of mission items.
     *
     * @param missionItemImpls list of waypoints to add
     */
    fun addMissionItems(missionItemImpls: List<MissionItemImpl?>?) {
        items.addAll(missionItemImpls!!)
        notifyMissionUpdate()
    }

    fun clearMissionItems() {
        items.clear()
        notifyMissionUpdate()
    }

    /**
     * Add a waypoint to the mission's set of mission item.
     *
     * @param missionItemImpl waypoint to add
     */
    fun addMissionItem(missionItemImpl: MissionItemImpl?) {
        items.add(missionItemImpl)
        notifyMissionUpdate()
    }

    fun addMissionItem(index: Int, missionItemImpl: MissionItemImpl?) {
        items.add(index, missionItemImpl)
        notifyMissionUpdate()
    }

    /**
     * Signals that this mission object was updated.
     */
    fun notifyMissionUpdate() {
        updateComponentItems()
        myDrone!!.notifyDroneEvent(DroneEventsType.MISSION_UPDATE)
    }

    /**
     * Updates a mission item
     *
     * @param oldItem mission item to update
     * @param newItem new mission item
     */
    fun replace(oldItem: MissionItemImpl?, newItem: MissionItemImpl?) {
        val index = items.indexOf(oldItem)
        if (index == -1) {
            return
        }
        items.removeAt(index)
        items.add(index, newItem)
        notifyMissionUpdate()
    }

    fun replaceAll(updatesList: List<Pair<MissionItemImpl?, MissionItemImpl?>>?) {
        if (updatesList == null || updatesList.isEmpty()) {
            return
        }
        var wasUpdated = false
        for (updatePair in updatesList) {
            val oldItem = updatePair.first
            val index = items.indexOf(oldItem)
            if (index == -1) {
                continue
            }
            val newItem = updatePair.second
            items.removeAt(index)
            items.add(index, newItem)
            wasUpdated = true
        }
        if (wasUpdated) {
            notifyMissionUpdate()
        }
    }

    /**
     * Reverse the order of the mission items.
     */
    fun reverse() {
        Collections.reverse(items)
        notifyMissionUpdate()
    }

    fun onWriteWaypoints(msg: msg_mission_ack?) {
        myDrone!!.notifyDroneEvent(DroneEventsType.MISSION_SENT)
    }

    fun getItems(): List<MissionItemImpl?> {
        return items
    }

    fun getComponentItems(): List<MissionItemImpl?> {
        return componentItems
    }

    fun getOrder(waypoint: MissionItemImpl?): Int {
        return items.indexOf(waypoint) + 1 // plus one to account for the fact
        // that this is an index
    }

    @Throws(IllegalArgumentException::class)
    fun getAltitudeDiffFromPreviousItem(waypoint: SpatialCoordItem): Double {
        val i = items.indexOf(waypoint)
        if (i > 0) {
            val previous = items[i - 1]
            if (previous is SpatialCoordItem) {
                return waypoint.coordinate.altitude - previous.coordinate
                        .altitude
            }
        }
        throw IllegalArgumentException("Last waypoint doesn't have an altitude")
    }

    @Throws(IllegalArgumentException::class)
    fun getDistanceFromLastWaypoint(waypoint: SpatialCoordItem): Double {
        val i = items.indexOf(waypoint)
        if (i > 0) {
            val previous = items[i - 1]
            if (previous is SpatialCoordItem) {
                return GeoTools.getDistance(waypoint.coordinate,
                        previous.coordinate)
            }
        }
        throw IllegalArgumentException("Last waypoint doesn't have a coordinate")
    }

    fun hasItem(item: MissionItemImpl?): Boolean {
        return items.contains(item)
    }

    fun onMissionReceived(msgs: List<msg_mission_item>?) {
        if (msgs != null) {
            myDrone!!.processHomeUpdate(msgs[0])
            (msgs as? MutableList)?.removeAt(0) // Remove Home waypoint
            items.clear()
            items.addAll(processMavLinkMessages(msgs))
            myDrone!!.notifyDroneEvent(DroneEventsType.MISSION_RECEIVED)
            notifyMissionUpdate()
        }
    }

    fun onMissionLoaded(msgs: List<msg_mission_item>?) {
        if (msgs != null) {
            myDrone!!.processHomeUpdate(msgs[0])
            (msgs as? MutableList)?.removeAt(0) // Remove Home waypoint
            items.clear()
            items.addAll(processMavLinkMessages(msgs))
            myDrone!!.notifyDroneEvent(DroneEventsType.MISSION_RECEIVED)
            notifyMissionUpdate()
        }
    }

    private fun processMavLinkMessages(msgs: List<msg_mission_item>): List<MissionItemImpl?> {
        val received: MutableList<MissionItemImpl?> = ArrayList()
        for (msg in msgs) {
            when (msg.command) {
                MAV_CMD.MAV_CMD_DO_SET_SERVO -> received.add(SetServoImpl(msg, this))
                MAV_CMD.MAV_CMD_NAV_WAYPOINT -> received.add(WaypointImpl(msg, this))
                MAV_CMD.MAV_CMD_NAV_SPLINE_WAYPOINT -> received.add(SplineWaypointImpl(msg, this))
                MAV_CMD.MAV_CMD_NAV_LAND -> received.add(LandImpl(msg, this))
                MAV_CMD.MAV_CMD_DO_LAND_START -> received.add(DoLandStartImpl(msg, this))
                MAV_CMD.MAV_CMD_NAV_TAKEOFF -> received.add(TakeoffImpl(msg, this))
                MAV_CMD.MAV_CMD_DO_CHANGE_SPEED -> received.add(ChangeSpeedImpl(msg, this))
                MAV_CMD.MAV_CMD_NAV_LOITER_TO_ALT -> received.add(LoiterToAltImpl(msg, this))
                MAV_CMD.MAV_CMD_NAV_LOITER_TIME -> received.add(LoiterTimeImpl(msg, this))
                MAV_CMD.MAV_CMD_DO_SET_CAM_TRIGG_DIST -> received.add(CameraTriggerImpl(msg, this))
                MAV_CMD.MAV_CMD_DO_GRIPPER -> received.add(EpmGripperImpl(msg, this))
                MAV_CMD.MAV_CMD_DO_SET_ROI, MAV_CMD.MAV_CMD_DO_SET_ROI_LOCATION, MAV_CMD.MAV_CMD_DO_SET_ROI_NONE -> received.add(RegionOfInterestImpl(msg, this))
                MAV_CMD.MAV_CMD_NAV_LOITER_TURNS -> received.add(CircleImpl(msg, this))
                MAV_CMD.MAV_CMD_NAV_RETURN_TO_LAUNCH -> received.add(ReturnToHomeImpl(msg, this))
                MAV_CMD.MAV_CMD_CONDITION_YAW -> received.add(ConditionYawImpl(msg, this))
                MAV_CMD.MAV_CMD_DO_SET_RELAY -> received.add(SetRelayImpl(msg, this))
                MAV_CMD.MAV_CMD_DO_JUMP -> received.add(DoJumpImpl(msg, this))
                MAV_CMD.MAV_CMD_NAV_VTOL_TAKEOFF -> received.add(VTOLTakeoffImpl(msg, this))
                MAV_CMD.MAV_CMD_NAV_VTOL_LAND -> received.add(VTOLLandImpl(msg, this))
                MAV_CMD.MAV_CMD_DO_VTOL_TRANSITION -> received.add(VTOLTransitionImpl(msg, this))
                else -> {}
            }
        }
        return received
    }

    /**
     * Sends the mission to the drone using the mavlink protocol.
     */
    fun sendMissionToAPM() {
        val msgMissionItems = msgMissionItems
        myDrone?.waypointManager?.writeWaypoints(msgMissionItems)
        updateComponentItems(msgMissionItems)
    }

    private fun updateComponentItems() {
        val msgMissionItems = msgMissionItems
        updateComponentItems(msgMissionItems)
    }

    private fun updateComponentItems(msgMissionItems: List<msg_mission_item>?) {
        componentItems.clear()
        if (msgMissionItems == null || msgMissionItems.isEmpty()) {
            return
        }
        val firstItem = msgMissionItems[0]
        if (firstItem.seq == APMConstants.HOME_WAYPOINT_INDEX) {
            (msgMissionItems as? MutableList)?.removeAt(0) // Remove Home waypoint
        }
        componentItems.addAll(processMavLinkMessages(msgMissionItems))
    }

    fun packHomeMavlink(): msg_mission_item {
        val home = myDrone!!.getAttribute(AttributeType.HOME) as Home?
        val coordinate = home!!.coordinate
        val mavMsg = msg_mission_item()
        mavMsg.autocontinue = 1
        mavMsg.command = MAV_CMD.MAV_CMD_NAV_WAYPOINT
        mavMsg.current = 0
        mavMsg.frame = MAV_FRAME.MAV_FRAME_GLOBAL.toShort()

        myDrone?.let {
            mavMsg.target_system = it.sysid
            mavMsg.target_component = it.compid
            if (home.isValid) {
                mavMsg.x = coordinate!!.latitude.toFloat()
                mavMsg.y = coordinate.longitude.toFloat()
                mavMsg.z = coordinate.altitude.toFloat()
            }
        }

        return mavMsg
    }

    private val msgMissionItems: List<msg_mission_item>
        get() {
            val data: MutableList<msg_mission_item> = ArrayList()
            var waypointCount = 0
            val home = packHomeMavlink()
            home.seq = waypointCount++
            data.add(home)
            val size = items.size
            for (i in 0 until size) {
                val item = items[i]
                for (msg_item in item!!.packMissionItem()) {
                    msg_item.seq = waypointCount++
                    msg_item.isMavlink2 = false
                    msg_item.mission_type = 0
                    data.add(msg_item)
                }
            }
            return data
        }

    /**
     * Create and upload a dronie mission to the drone
     *
     * @return the bearing in degrees the drone trajectory will take.
     */
    fun makeAndUploadDronie(): Double {
        val droneGps = myDrone!!.getAttribute(AttributeType.GPS) as Gps?
        val currentPosition = droneGps!!.position
        if (currentPosition == null || droneGps.satellitesCount <= 5) {
            myDrone!!.notifyDroneEvent(DroneEventsType.WARNING_NO_GPS)
            return (-1).toDouble()
        }
        val attitude = myDrone!!.getAttribute(AttributeType.ATTITUDE) as Attitude?
        val bearing = 180 + attitude!!.yaw
        items.clear()
        items.addAll(createDronie(currentPosition,
                GeoTools.newCoordFromBearingAndDistance(currentPosition, bearing, 50.0)))
        sendMissionToAPM()
        notifyMissionUpdate()
        return bearing
    }

    private val speedParameter: Double
        private get() {
            val param = myDrone?.parameterManager?.getParameter("WPNAV_SPEED")
            return if (param == null) {
                (-1).toDouble()
            } else {
                param.value / 100
            }
        }

    fun createDronie(start: LatLong?, end: LatLong?): List<MissionItemImpl?> {
        val startAltitude = 4.0
        val roiDistance = -8
        val slowDownPoint = GeoTools.pointAlongTheLine(start, end, 5)
        var defaultSpeed = speedParameter
        if (defaultSpeed == -1.0) {
            defaultSpeed = 5.0
        }
        val dronieItems: MutableList<MissionItemImpl?> = ArrayList()
        dronieItems.add(TakeoffImpl(this, startAltitude))
        dronieItems.add(RegionOfInterestImpl(this,
                LatLongAlt(GeoTools.pointAlongTheLine(start, end, roiDistance), 1.0)))
        dronieItems.add(WaypointImpl(this, LatLongAlt(end, startAltitude + GeoTools.getDistance(start, end) / 2.0)))
        dronieItems.add(WaypointImpl(this,
                LatLongAlt(slowDownPoint, startAltitude + GeoTools.getDistance(start, slowDownPoint) / 2.0)))
        dronieItems.add(ChangeSpeedImpl(this, 1.0))
        dronieItems.add(WaypointImpl(this, LatLongAlt(start, startAltitude.toDouble())))
        dronieItems.add(ChangeSpeedImpl(this, defaultSpeed))
        dronieItems.add(LandImpl(this, start))
        return dronieItems
    }

    fun hasTakeoffAndLandOrRTL(): Boolean {
        if (items.size >= 2) {
            if (isFirstItemTakeoff && isLastItemLandOrRTL) {
                return true
            }
        }
        return false
    }

    val isFirstItemTakeoff: Boolean
        get() = !items.isEmpty() && items[0] is TakeoffImpl
    val isLastItemLandOrRTL: Boolean
        get() {
            if (items.isEmpty()) return false
            val last = items[items.size - 1]
            return last is ReturnToHomeImpl || last is LandImpl
        }
}

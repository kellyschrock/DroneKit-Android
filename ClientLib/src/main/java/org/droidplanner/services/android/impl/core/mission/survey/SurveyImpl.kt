package org.droidplanner.services.android.impl.core.mission.survey

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import com.MAVLink.enums.MAV_FRAME
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.util.MathUtils.getDistance2D
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType
import org.droidplanner.services.android.impl.core.mission.commands.CameraTriggerImpl
import org.droidplanner.services.android.impl.core.polygon.Polygon
import org.droidplanner.services.android.impl.core.survey.CameraInfo
import org.droidplanner.services.android.impl.core.survey.SurveyData
import org.droidplanner.services.android.impl.core.survey.grid.Grid
import org.droidplanner.services.android.impl.core.survey.grid.GridBuilder
import timber.log.Timber
import java.util.*

open class SurveyImpl(mission: Mission?, points: List<LatLong>) : MissionItemImpl(mission) {
    @JvmField
    var polygon = Polygon()
    @JvmField
    var surveyData = SurveyData()
    private val cameraElevations = ArrayList<LatLongAlt>()
    var centerElevation = 0.0
    @JvmField
    var grid: Grid? = null
    var isStartCameraBeforeFirstWaypoint = false

    fun getCameraElevations(): List<LatLongAlt> {
        return cameraElevations
    }

    fun setCameraElevations(elevations: List<LatLongAlt>?) {
        cameraElevations.clear()
        cameraElevations.addAll(elevations!!)
    }

    fun update(angle: Double, altitude: Double, overlap: Double, sidelap: Double, lockOrientation: Boolean, lockYaw: Boolean, lockYawAngle: Double) {
        surveyData.update(angle, altitude, overlap, sidelap, lockOrientation, lockYaw, lockYawAngle)
    }

    fun setCameraInfo(camera: CameraInfo) {
        surveyData.cameraInfo = camera
    }

    @Throws(Exception::class)
    fun build() {
        // TODO find better point than (0,0) to reference the grid
        grid = null
        val gridBuilder = GridBuilder(polygon, surveyData, LatLong(0.0, 0.0))
        polygon.checkIfValid()
        grid = gridBuilder.generate(true)
    }

    override fun packMissionItem(): List<msg_mission_item> {
        return try {
            val list: MutableList<msg_mission_item> = ArrayList()
            build()
            packSurveyPoints(list)
            list
        } catch (e: Exception) {
            ArrayList()
        }
    }

    private fun packSurveyPoints(list: MutableList<msg_mission_item>) {
        //Generate the camera trigger
        val camTrigger = CameraTriggerImpl(mission, surveyData.longitudinalPictureDistance)

        //Add it if the user wants it to start before the first waypoint.
        if (isStartCameraBeforeFirstWaypoint) {
            list.addAll(camTrigger.packMissionItem())
        }
        val altitude = surveyData.getAltitude()
        Timber.d("packSurveyPoints(): centerElevation=%.2f lockYaw=%s lockYawAngle=%.2f", centerElevation, surveyData.lockYaw, surveyData.lockYawAngle)

        //Add the camera trigger after the first waypoint if it wasn't added before.
        var addToFirst = !isStartCameraBeforeFirstWaypoint
        for (point in grid!!.gridPoints) {
            val mavMsg = getSurveyPoint(point, altitude)
            list.add(mavMsg)
            if (surveyData.lockOrientation) {
                val yawMsg = getYawCondition(surveyData.angle!!)
                list.add(yawMsg)
            } else if (surveyData.lockYaw) {
                val yawMsg = getYawCondition(surveyData.lockYawAngle)
                list.add(yawMsg)
            }
            if (addToFirst) {
                list.addAll(camTrigger.packMissionItem())
                addToFirst = false
            }
        }
        list.addAll(CameraTriggerImpl(mission, 0.0).packMissionItem())
    }

    protected open fun getSurveyPoint(point: LatLong, altitude: Double): msg_mission_item {
        var altitude = altitude
        val nearest = findNearestElevation(cameraElevations, point)
        if (nearest != null && centerElevation > 0) {
            val elevation = nearest.altitude
            val diff = elevation - centerElevation
            val newAlt = altitude + diff
            Timber.d("point=%s elevation=%.2f alt=%.2f newAlt=%.2f", point, elevation, altitude, newAlt)
            altitude = newAlt
        }
        return packSurveyPoint(point, altitude)
    }

    private fun getYawCondition(angle: Double): msg_mission_item {
        val mavMsg = msg_mission_item()
        mavMsg.autocontinue = 1
        mavMsg.frame = MAV_FRAME.MAV_FRAME_LOCAL_ENU.toShort()
        mavMsg.command = MAV_CMD.MAV_CMD_CONDITION_YAW
        mavMsg.x = 0f
        mavMsg.y = 0f
        mavMsg.z = 0f
        mavMsg.param1 = angle.toFloat()
        //yaw craft a 30 degrees per second (value is relatively insignificant since it only applies when approaching first waypoint of mission)
        mavMsg.param2 = 30f
        mavMsg.param3 = 0f
        mavMsg.param4 = 0f
        return mavMsg
    }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {}
    override fun getType(): MissionItemType {
        return MissionItemType.SURVEY
    }

    companion object {
        fun packSurveyPoint(point: LatLong, altitude: Double): msg_mission_item {
            val mavMsg = msg_mission_item()
            mavMsg.autocontinue = 1
            mavMsg.frame = MAV_FRAME.MAV_FRAME_GLOBAL_RELATIVE_ALT.toShort()
            mavMsg.command = MAV_CMD.MAV_CMD_NAV_WAYPOINT
            mavMsg.x = point.latitude.toFloat()
            mavMsg.y = point.longitude.toFloat()
            mavMsg.z = altitude.toFloat()
            mavMsg.param1 = 0f
            mavMsg.param2 = 0f
            mavMsg.param3 = 0f
            mavMsg.param4 = 0f
            return mavMsg
        }

        fun findNearestElevation(results: List<LatLongAlt>, location: LatLong?): LatLongAlt? {
            Collections.sort(results) { o1, o2 ->
                val ll1: LatLong? = o1
                val ll2: LatLong? = o2
                if (ll1 != null && ll2 != null) {
                    val d1 = getDistance2D(location, ll1)
                    val d2 = getDistance2D(location, ll2)
                    if (d1 < d2) {
                        -1
                    } else if (d2 < d1) {
                        1
                    } else {
                        0
                    }
                } else {
                    0
                }
            }
            return if (results.isEmpty()) null else results[0]
        }
    }

    init {
        polygon.addPoints(points)
    }
}

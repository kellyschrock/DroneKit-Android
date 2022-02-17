package org.droidplanner.services.android.impl.core.mission.waypoints

import com.MAVLink.common.msg_mission_item
import com.MAVLink.enums.MAV_CMD
import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import org.droidplanner.services.android.impl.core.helpers.geoTools.GeoTools
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.MissionItemType
import org.droidplanner.services.android.impl.core.mission.survey.SurveyImpl
import org.droidplanner.services.android.impl.core.polygon.Polygon
import org.droidplanner.services.android.impl.core.survey.CameraInfo
import org.droidplanner.services.android.impl.core.survey.SurveyData
import org.droidplanner.services.android.impl.core.survey.grid.GridBuilder
import java.util.*

class StructureScannerImpl : SpatialCoordItem {
    var radius = 10.0
        private set
    var endAltitude = 5.toDouble()
        private set
    var numberOfSteps = 2
    var isCrossHatchEnabled = false
        private set
    var surveyData = SurveyData()

    constructor(mission: Mission?, coord: LatLongAlt?) : super(mission, coord) {}
    constructor(item: MissionItemImpl?) : super(item) {}

    override fun packMissionItem(): List<msg_mission_item> {
        val list: MutableList<msg_mission_item> = ArrayList()
        packROI(list)
        packCircles(list)
        if (isCrossHatchEnabled) {
            packHatch(list)
        }
        return list
    }

    private fun packROI(list: MutableList<msg_mission_item>) {
        val roi = RegionOfInterestImpl(mission, LatLongAlt(coordinate, 0.0))
        list.addAll(roi.packMissionItem())
    }

    private fun packCircles(list: MutableList<msg_mission_item>) {
        var altitude = coordinate.altitude
        while (altitude <= topHeight) {
            val circleImpl = CircleImpl(mission, LatLongAlt(coordinate, altitude))
            circleImpl.radius = radius
            list.addAll(circleImpl.packMissionItem())
            altitude += endAltitude
        }
    }

    private fun packHatch(list: MutableList<msg_mission_item>) {
        val polygon = Polygon()
        var angle = 0.0
        while (angle <= 360) {
            polygon.addPoint(GeoTools.newCoordFromBearingAndDistance(coordinate, angle, radius))
            angle += 10.0
        }
        val corner = GeoTools.newCoordFromBearingAndDistance(coordinate, -45.0, radius * 2)
        surveyData.setAltitude(topHeight)
        try {
            surveyData.update(0.0, surveyData.getAltitude(), surveyData.getOverlap(), surveyData.getSidelap(), surveyData.lockOrientation, false, 0.0)
            val grid = GridBuilder(polygon, surveyData, corner)
            for (point in grid.generate(false).gridPoints) {
                list.add(SurveyImpl.packSurveyPoint(point, topHeight))
            }
            surveyData.update(90.0, surveyData.getAltitude(), surveyData.getOverlap(), surveyData.getSidelap(), surveyData.lockOrientation, false, 0.0)
            val grid2 = GridBuilder(polygon, surveyData, corner)
            for (point in grid2.generate(false).gridPoints) {
                list.add(SurveyImpl.packSurveyPoint(point, topHeight))
            }
        } catch (e: Exception) { // Should never fail, since it has good polygons
        }
    }

    val path: List<LatLong>
        get() {
            val path: MutableList<LatLong> = ArrayList()
            for (msg_mission_item in packMissionItem()) {
                if (msg_mission_item.command == MAV_CMD.MAV_CMD_NAV_WAYPOINT) {
                    path.add(LatLong(msg_mission_item.x.toDouble(), msg_mission_item.y.toDouble()))
                }
                if (msg_mission_item.command == MAV_CMD.MAV_CMD_NAV_LOITER_TURNS) {
                    var angle = 0.0
                    while (angle <= 360) {
                        path.add(GeoTools.newCoordFromBearingAndDistance(coordinate, angle, radius))
                        angle += 12.0
                    }
                }
            }
            return path
        }

    override fun unpackMAVMessage(mavMsg: msg_mission_item) {}
    override fun getType(): MissionItemType = MissionItemType.CYLINDRICAL_SURVEY

    private val topHeight: Double
        private get() = coordinate.altitude + (numberOfSteps - 1) * endAltitude
    val center: LatLong
        get() = coordinate

    fun setRadius(newValue: Int) {
        radius = newValue.toDouble()
    }

    fun enableCrossHatch(isEnabled: Boolean) {
        isCrossHatchEnabled = isEnabled
    }

    fun setAltitudeStep(newValue: Int) {
        endAltitude = newValue.toDouble()
    }

    fun setCamera(cameraInfo: CameraInfo) {
        surveyData.cameraInfo = cameraInfo
    }

    val camera: String
        get() = surveyData.camera.cameraName
}

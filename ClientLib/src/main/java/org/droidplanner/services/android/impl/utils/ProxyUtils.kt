package org.droidplanner.services.android.impl.utils

import android.util.Log
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.drone.mission.MissionItemType
import com.o3dr.services.android.lib.drone.mission.item.MissionItem
import com.o3dr.services.android.lib.drone.mission.item.command.*
import com.o3dr.services.android.lib.drone.mission.item.complex.*
import com.o3dr.services.android.lib.drone.mission.item.spatial.*
import org.droidplanner.services.android.impl.core.mission.Mission
import org.droidplanner.services.android.impl.core.mission.MissionItemImpl
import org.droidplanner.services.android.impl.core.mission.commands.*
import org.droidplanner.services.android.impl.core.mission.survey.SplineSurveyImpl
import org.droidplanner.services.android.impl.core.mission.survey.SurveyImpl
import org.droidplanner.services.android.impl.core.mission.waypoints.*
import org.droidplanner.services.android.impl.core.survey.CameraInfo
import org.droidplanner.services.android.impl.core.survey.SurveyData

/**
 * Created by fhuya on 11/10/14.
 */
object ProxyUtils {
    private val TAG = ProxyUtils::class.java.simpleName
    @JvmStatic
    fun getCameraDetail(camInfo: CameraInfo?): CameraDetail? {
        return if (camInfo == null) null else CameraDetail(camInfo.name, camInfo.sensorWidth,
                camInfo.sensorHeight, camInfo.sensorResolution, camInfo.focalLength,
                camInfo.overlap, camInfo.sidelap, camInfo.isInLandscapeOrientation)
    }

    fun getCameraInfo(camDetail: CameraDetail?): CameraInfo? {
        if (camDetail == null) return null
        val camInfo = CameraInfo()
        camInfo.name = camDetail.name
        camInfo.sensorWidth = camDetail.sensorWidth
        camInfo.sensorHeight = camDetail.sensorHeight
        camInfo.sensorResolution = camDetail.sensorResolution
        camInfo.focalLength = camDetail.focalLength
        camInfo.overlap = camDetail.overlap
        camInfo.sidelap = camDetail.sidelap
        camInfo.isInLandscapeOrientation = camDetail.isInLandscapeOrientation
        return camInfo
    }

    fun getSurveyDetail(surveyData: SurveyData): SurveyDetail {
        val surveyDetail = SurveyDetail()
        surveyDetail.cameraDetail = getCameraDetail(surveyData.cameraInfo)
        surveyDetail.sidelap = surveyData.sidelap
        surveyDetail.overlap = surveyData.overlap
        surveyDetail.angle = surveyData.angle
        surveyDetail.altitude = surveyData.altitude
        surveyDetail.lockOrientation = surveyData.lockOrientation
        surveyDetail.lockYaw = surveyData.lockYaw
        surveyDetail.lockYawAngle = surveyData.lockYawAngle
        return surveyDetail
    }

    @JvmStatic
    fun getMissionItemImpl(mission: Mission?, proxyItem: MissionItem?): MissionItemImpl? {
        if (proxyItem == null) return null
        val missionItemImpl: MissionItemImpl?
        when (proxyItem.type) {
            MissionItemType.CAMERA_TRIGGER -> {
                val proxy = proxyItem as CameraTrigger
                val temp = CameraTriggerImpl(mission, proxy.triggerDistance)
                missionItemImpl = temp
            }
            MissionItemType.CHANGE_SPEED -> {
                val proxy = proxyItem as ChangeSpeed
                val temp = ChangeSpeedImpl(mission, proxy.speed)
                missionItemImpl = temp
            }
            MissionItemType.LOITER_TO_ALT -> {
                val proxy = proxyItem as LoiterToAlt
                val coord = proxy.coordinate
                missionItemImpl = if (coord != null) {
                    val impl = LoiterToAltImpl(mission, coord.latitude, coord.longitude, coord.altitude)
                    impl
                } else {
                    null
                }
            }
            MissionItemType.LOITER_TIME -> {
                val proxy = proxyItem as LoiterTime
                val coord = proxy.coordinate
                missionItemImpl = if (coord != null) {
                    val impl = LoiterTimeImpl(mission, coord.latitude, coord.longitude, coord.altitude, proxy.delay, proxy.radius)
                    impl
                } else {
                    null
                }
            }
            MissionItemType.VTOL_TAKEOFF -> {
                val proxy = proxyItem as VTOLTakeoff
                val coord = proxy.coordinate
                missionItemImpl = if (coord != null) {
                    VTOLTakeoffImpl(
                            mission, proxy.transitionHeading.value, proxy.yawAngle,
                            coord.latitude, coord.longitude, coord.altitude)
                } else {
                    null
                }
            }
            MissionItemType.VTOL_TRANSITION -> {
                val proxy = proxyItem as VTOLTransition
                missionItemImpl = VTOLTransitionImpl(mission, proxy.targetState.state)
            }
            MissionItemType.VTOL_LAND -> {
                val proxy = proxyItem as VTOLLand
                val coord = proxy.coordinate
                missionItemImpl = if (coord != null) {
                    VTOLLandImpl(mission, proxy.approachAltitude, proxy.yawAngle, coord.latitude, coord.longitude, coord.altitude)
                } else {
                    null
                }
            }
            MissionItemType.RAW_COMMAND -> {
                val proxy = proxyItem as RawMissionCommand
                missionItemImpl = RawMissionCommandImpl(mission).setTo(proxy)
            }
            MissionItemType.TAKE_PICTURE -> {
                val proxy = proxyItem as TakePicture
                missionItemImpl = TakePictureImpl(mission, 0.0)
            }
            MissionItemType.EPM_GRIPPER -> {
                val proxy = proxyItem as EpmGripper
                val temp = EpmGripperImpl(mission, proxy.isRelease)
                missionItemImpl = temp
            }
            MissionItemType.RETURN_TO_LAUNCH -> {
                val proxy = proxyItem as ReturnToLaunch
                val temp = ReturnToHomeImpl(mission)
                temp.height = proxy.returnAltitude
                missionItemImpl = temp
            }
            MissionItemType.SET_SERVO -> {
                val proxy = proxyItem as SetServo
                val temp = SetServoImpl(mission, proxy.channel, proxy.pwm)
                missionItemImpl = temp
            }
            MissionItemType.TAKEOFF -> {
                val proxy = proxyItem as Takeoff
                val temp = TakeoffImpl(mission, proxy.takeoffAltitude, proxy.takeoffPitch)
                missionItemImpl = temp
            }
            MissionItemType.CIRCLE -> {
                val proxy = proxyItem as Circle
                val temp = CircleImpl(mission, proxy.coordinate)
                temp.radius = proxy.radius
                temp.setTurns(proxy.turns)
                missionItemImpl = temp
            }
            MissionItemType.LAND -> {
                val proxy = proxyItem as Land
                val temp = LandImpl(mission, proxy.coordinate)
                missionItemImpl = temp
            }
            MissionItemType.DO_LAND_START -> {
                val proxy = proxyItem as DoLandStart
                val temp = DoLandStartImpl(mission, proxy.coordinate)
                missionItemImpl = temp
            }
            MissionItemType.REGION_OF_INTEREST -> {
                val proxy = proxyItem as RegionOfInterest
                val temp = RegionOfInterestImpl(mission, proxy.coordinate)
                missionItemImpl = temp
            }
            MissionItemType.RESET_ROI -> {

                //Sending a roi with all coordinates set to 0 will reset the current roi.
                val temp = RegionOfInterestImpl(mission, LatLongAlt(0.0, 0.0, 0.0))
                missionItemImpl = temp
            }
            MissionItemType.SPLINE_WAYPOINT -> {
                val proxy = proxyItem as SplineWaypoint
                val temp = SplineWaypointImpl(mission, proxy.coordinate)
                temp.delay = proxy.delay
                missionItemImpl = temp
            }
            MissionItemType.STRUCTURE_SCANNER -> {
                val proxy = proxyItem as StructureScanner
                val temp = StructureScannerImpl(mission, proxy.coordinate)
                temp.setRadius(proxy.radius.toInt())
                temp.numberOfSteps = proxy.stepsCount
                temp.setAltitudeStep(proxy.heightStep.toInt())
                temp.enableCrossHatch(proxy.isCrossHatch)
                val camDetail = proxy.surveyDetail!!.cameraDetail
                if (camDetail != null) temp.setCamera(getCameraInfo(camDetail))
                missionItemImpl = temp
            }
            MissionItemType.WAYPOINT -> {
                val proxy = proxyItem as Waypoint
                val temp = WaypointImpl(mission, proxy
                        .coordinate)
                temp.acceptanceRadius = proxy.acceptanceRadius
                temp.delay = proxy.delay
                temp.isOrbitCCW = proxy.isOrbitCCW
                temp.orbitalRadius = proxy.orbitalRadius
                temp.yawAngle = proxy.yawAngle
                missionItemImpl = temp
            }
            MissionItemType.SURVEY -> {
                val proxy = proxyItem as Survey
                val surveyDetail = proxy.surveyDetail
                val temp = SurveyImpl(mission, proxy.polygonPoints)
                temp.isStartCameraBeforeFirstWaypoint = proxy.isStartCameraBeforeFirstWaypoint
                temp.setCameraElevations(proxy.getCameraElevations())
                temp.centerElevation = proxy.centerElevation
                if (surveyDetail != null) {
                    val cameraDetail = surveyDetail.cameraDetail
                    if (cameraDetail != null) temp.setCameraInfo(getCameraInfo(cameraDetail))
                    temp.update(surveyDetail.angle, surveyDetail.altitude,
                            surveyDetail.overlap, surveyDetail.sidelap,
                            surveyDetail.lockOrientation,
                            surveyDetail.lockYaw, surveyDetail.lockYawAngle)
                }
                try {
                    temp.build()
                } catch (e: Exception) {
                    Log.e(TAG, e.message, e)
                }
                missionItemImpl = temp
            }
            MissionItemType.SPLINE_SURVEY -> {
                val proxy = proxyItem as SplineSurvey
                val surveyDetail = proxy.surveyDetail
                val temp = SplineSurveyImpl(mission, proxy.polygonPoints)
                temp.isStartCameraBeforeFirstWaypoint = proxy.isStartCameraBeforeFirstWaypoint
                temp.setCameraElevations(proxy.getCameraElevations())
                if (surveyDetail != null) {
                    val cameraDetail = surveyDetail.cameraDetail
                    if (cameraDetail != null) temp.setCameraInfo(getCameraInfo(cameraDetail))
                    temp.update(surveyDetail.angle, surveyDetail.altitude,
                            surveyDetail.overlap, surveyDetail.sidelap, surveyDetail.lockOrientation,
                            surveyDetail.lockYaw, surveyDetail.lockYawAngle)
                }
                try {
                    temp.build()
                } catch (e: Exception) {
                    Log.e(TAG, e.message, e)
                }
                missionItemImpl = temp
            }
            MissionItemType.YAW_CONDITION -> {
                val proxy = proxyItem as YawCondition
                val temp = ConditionYawImpl(mission, proxy.angle, proxy.isRelative)
                temp.angularSpeed = proxy.angularSpeed
                missionItemImpl = temp
            }
            MissionItemType.SET_RELAY -> {
                val proxy = proxyItem as SetRelay
                missionItemImpl = SetRelayImpl(mission, proxy.relayNumber, proxy.isEnabled)
            }
            MissionItemType.DO_JUMP -> {
                val proxy = proxyItem as DoJump
                missionItemImpl = DoJumpImpl(mission, proxy.waypoint, proxy.repeatCount)
            }
            else -> missionItemImpl = null
        }
        return missionItemImpl
    }

    @JvmStatic
    fun getProxyMissionItem(itemImpl: MissionItemImpl?): MissionItem? {
        if (itemImpl == null) return null
        val proxyMissionItem: MissionItem?
        when (itemImpl.type) {
            org.droidplanner.services.android.impl.core.mission.MissionItemType.WAYPOINT -> {
                val source = itemImpl as WaypointImpl
                val temp = Waypoint()
                temp.coordinate = source.coordinate
                temp.acceptanceRadius = source.acceptanceRadius
                temp.delay = source.delay
                temp.orbitalRadius = source.orbitalRadius
                temp.isOrbitCCW = source.isOrbitCCW
                temp.yawAngle = source.yawAngle
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.SPLINE_WAYPOINT -> {
                val source = itemImpl as SplineWaypointImpl
                val temp = SplineWaypoint()
                temp.coordinate = source.coordinate
                temp.delay = source.delay
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.TAKEOFF -> {
                val source = itemImpl as TakeoffImpl
                val temp = Takeoff()
                temp.takeoffAltitude = source.finishedAlt
                temp.takeoffPitch = source.pitch
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.RTL -> {
                val source = itemImpl as ReturnToHomeImpl
                val temp = ReturnToLaunch()
                temp.returnAltitude = source.height
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.LAND -> {
                val source = itemImpl as LandImpl
                val temp = Land()
                temp.coordinate = source.coordinate
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.DO_LAND_START -> {
                val source = itemImpl as DoLandStartImpl
                val temp = DoLandStart()
                temp.coordinate = source.coordinate
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.CIRCLE -> {
                val source = itemImpl as CircleImpl
                val temp = Circle()
                temp.coordinate = source.coordinate
                temp.radius = source.radius
                temp.turns = source.numberOfTurns
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.ROI -> {
                val source = itemImpl as RegionOfInterestImpl
                if (source.isReset) {
                    val temp = ResetROI()
                    proxyMissionItem = temp
                } else {
                    val temp = RegionOfInterest()
                    temp.coordinate = source.coordinate
                    proxyMissionItem = temp
                }
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.SURVEY -> {
                val source = itemImpl as SurveyImpl
                var isValid = true
                try {
                    source.build()
                } catch (e: Exception) {
                    isValid = false
                }
                val temp = Survey()
                temp.isStartCameraBeforeFirstWaypoint = source.isStartCameraBeforeFirstWaypoint
                temp.isValid = isValid
                temp.surveyDetail = getSurveyDetail(source.surveyData)
                temp.polygonPoints = source.polygon.points
                temp.setCameraElevations(source.getCameraElevations())
                temp.centerElevation = source.centerElevation
                if (source.grid != null) {
                    temp.gridPoints = source.grid!!.gridPoints
                    temp.cameraLocations = source.grid!!.cameraLocations
                }
                temp.polygonArea = source.polygon.area.valueInSqMeters()
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.SPLINE_SURVEY -> {
                val source = itemImpl as SplineSurveyImpl
                var isValid = true
                try {
                    source.build()
                } catch (e: Exception) {
                    isValid = false
                }
                val temp = Survey()
                temp.isStartCameraBeforeFirstWaypoint = source.isStartCameraBeforeFirstWaypoint
                temp.isValid = isValid
                temp.surveyDetail = getSurveyDetail(source.surveyData)
                temp.polygonPoints = source.polygon.points
                temp.setCameraElevations(source.getCameraElevations())
                if (source.grid != null) {
                    temp.gridPoints = source.grid!!.gridPoints
                    temp.cameraLocations = source.grid!!.cameraLocations
                }
                temp.polygonArea = source.polygon.area.valueInSqMeters()
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.CYLINDRICAL_SURVEY -> {
                val source = itemImpl as StructureScannerImpl
                val temp = StructureScanner()
                temp.surveyDetail = getSurveyDetail(source.surveyData)
                temp.coordinate = source.coordinate
                temp.radius = source.radius
                temp.isCrossHatch = source.isCrossHatchEnabled
                temp.heightStep = source.endAltitude
                temp.stepsCount = source.numberOfSteps
                temp.path = source.path
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.CHANGE_SPEED -> {
                val source = itemImpl as ChangeSpeedImpl
                val temp = ChangeSpeed()
                temp.speed = source.speed
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.LOITER_TO_ALT -> {
                val source = itemImpl as LoiterToAltImpl
                val temp = LoiterToAlt()
                temp.coordinate = LatLongAlt(source.lat, source.lng, source.alt)
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.LOITER_TIME -> {
                val source = itemImpl as LoiterTimeImpl
                val tmp = LoiterTime()
                tmp.coordinate = LatLongAlt(source.lat, source.lng, source.alt)
                tmp.delay = source.delay
                tmp.radius = source.radius
                proxyMissionItem = tmp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.VTOL_TAKEOFF -> {
                val impl = itemImpl as VTOLTakeoffImpl
                val proxy = VTOLTakeoff()
                proxy.coordinate = LatLongAlt(impl.lat, impl.lng, impl.alt)
                proxy.transitionHeading = VTOLTakeoff.TransitionHeading.fromValue(impl.frontTransitionHeading)
                proxy.yawAngle = impl.yawAngle
                proxyMissionItem = proxy
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.VTOL_TRANSITION -> {
                val impl = itemImpl as VTOLTransitionImpl
                val p = VTOLTransition()
                p.targetState = VTOLTransition.TargetState.fromValue(impl.targetState)
                proxyMissionItem = p
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.VTOL_LAND -> {
                val impl = itemImpl as VTOLLandImpl
                val p = VTOLLand()
                p.coordinate = LatLongAlt(impl.lat, impl.lng, impl.alt)
                p.approachAltitude = impl.approachAltitude
                p.yawAngle = impl.yawAngle
                proxyMissionItem = p
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.RAW_COMMAND -> {
                val impl = itemImpl as RawMissionCommandImpl
                val p = RawMissionCommand()
                p.setTo(impl)
                proxyMissionItem = p
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.TAKE_PICTURE -> {
                proxyMissionItem = TakePicture()
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.CAMERA_TRIGGER -> {
                val source = itemImpl as CameraTriggerImpl
                val temp = CameraTrigger()
                temp.triggerDistance = source.triggerDistance
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.EPM_GRIPPER -> {
                val source = itemImpl as EpmGripperImpl
                val temp = EpmGripper()
                temp.isRelease = source.isRelease
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.SET_SERVO -> {
                val source = itemImpl as SetServoImpl
                val temp = SetServo()
                temp.channel = source.channel
                temp.pwm = source.pwm
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.CONDITION_YAW -> {
                val source = itemImpl as ConditionYawImpl
                val temp = YawCondition()
                temp.angle = source.angle
                temp.angularSpeed = source.angularSpeed
                temp.isRelative = source.isRelative
                proxyMissionItem = temp
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.SET_RELAY -> {
                val impl = itemImpl as SetRelayImpl
                val proxy = SetRelay()
                proxy.relayNumber = impl.relayNumber
                proxy.isEnabled = impl.isEnabled
                proxyMissionItem = proxy
            }
            org.droidplanner.services.android.impl.core.mission.MissionItemType.DO_JUMP -> {
                val source = itemImpl as DoJumpImpl
                val proxy = DoJump()
                proxy.waypoint = source.waypoint
                proxy.repeatCount = source.repeatCount
                proxyMissionItem = proxy
            }
            else -> proxyMissionItem = null
        }
        return proxyMissionItem
    }
}

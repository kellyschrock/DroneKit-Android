package org.droidplanner.services.android.impl.core.survey

import org.droidplanner.services.android.impl.core.helpers.units.Area
import java.util.*

class SurveyData {
    var camera = CameraInfo()
    var angle: Double? = null
        private set
    var overlap: Double? = null
    var sidelap: Double? = null
    var lockOrientation = false
    private var footprint: Footprint? = null
    var lockYaw = false
    var lockYawAngle = 0.0

    private var altitude = 0.0


    fun update(angle: Double, altitude: Double, overlap: Double, sidelap: Double, lockOrientation: Boolean, lockYaw: Boolean, lockYawAngle: Double) {
        this.angle = angle
        this.overlap = overlap
        this.sidelap = sidelap
        setAltitude(altitude)
        this.lockOrientation = lockOrientation
        this.lockYaw = lockYaw
        this.lockYawAngle = lockYawAngle
    }

    fun setAltitude(altitude: Double) {
        this.altitude = altitude
        footprint = Footprint(camera, this.altitude)
    }

    var cameraInfo: CameraInfo
        get() = camera
        set(info) {
            camera = info
            footprint = Footprint(camera, altitude)
            tryToLoadOverlapFromCamera()
        }

    private fun tryToLoadOverlapFromCamera() {
        if (camera.overlap > 0.0) {
            overlap = camera.overlap
        }
        if (camera.sidelap > 0.0) {
            sidelap = camera.sidelap
        }
    }

    val longitudinalPictureDistance: Double
        get() = footprint!!.longitudinalFootPrint * (1 - overlap!! * .01)

    val lateralPictureDistance: Double
        get() = footprint!!.lateralFootPrint * (1 - sidelap!! * .01)

    fun getAltitude(): Double {
        return altitude
    }

    fun getSidelap(): Double {
        return sidelap!!
    }

    fun getOverlap(): Double {
        return overlap!!
    }

    val groundResolution: Area
        get() = Area(footprint!!.gSD * 0.01)

    override fun toString(): String {
        return String.format(Locale.US, "Altitude: %f Angle %f Overlap: %f Sidelap: %f Locked Orientation: %b, lockYaw=%s, lockYawAngle=%f", altitude,
                angle, overlap, sidelap, lockOrientation, lockYaw, lockYawAngle)
    }

    init {
        update(0.0, 50.0, 50.0, 60.0, false, false, 0.0)
    }
}

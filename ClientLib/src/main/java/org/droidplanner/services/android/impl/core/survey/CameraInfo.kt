package org.droidplanner.services.android.impl.core.survey

class CameraInfo {
    @JvmField
	var cameraName = "Canon SX260"
    @JvmField
	var sensorWidth = 6.12
    @JvmField
	var sensorHeight = 4.22
    @JvmField
	var sensorResolution = 12.1
    @JvmField
	var focalLength = 5.0
    @JvmField
	var overlap = 50.0
    @JvmField
	var sidelap = 60.0

    var name: String
        get() = cameraName
        set(value) { cameraName = value }

    var isInLandscapeOrientation = true

    val sensorLateralSize: Double
        get() = if (isInLandscapeOrientation) {
            sensorWidth
        } else {
            sensorHeight
        }

    val sensorLongitudinalSize: Double
        get() = if (isInLandscapeOrientation) {
            sensorHeight
        } else {
            sensorWidth
        }

    override fun toString(): String {
        return ("Camera:" + cameraName + " ImageWidth:" + sensorWidth + " ImageHeight:" + sensorHeight + " FocalLength:"
                + focalLength + " Overlap:" + overlap + " Sidelap:" + sidelap
                + " isInLandscapeOrientation:" + isInLandscapeOrientation)
    }
}

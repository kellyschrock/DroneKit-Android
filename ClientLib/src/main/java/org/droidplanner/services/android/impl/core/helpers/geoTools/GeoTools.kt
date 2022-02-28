package org.droidplanner.services.android.impl.core.helpers.geoTools

import com.o3dr.services.android.lib.coordinate.LatLong
import com.o3dr.services.android.lib.coordinate.LatLongAlt
import com.o3dr.services.android.lib.util.MathUtils.hypot
import org.droidplanner.services.android.impl.core.helpers.units.Area
import org.droidplanner.services.android.impl.core.polygon.Polygon

class GeoTools {
    // Source: WGS84
    var waypoints: List<LatLong>? = null

    companion object {
        private const val RADIUS_OF_EARTH = 6378137.0 // In meters.

        /**
         * Returns the distance between two points
         *
         * @return distance between the points in degrees
         */
        @JvmStatic
        fun getAproximatedDistance(p1: LatLong, p2: LatLong): Double {
            return Math.hypot(p1.latitude - p2.latitude, p1.longitude - p2.longitude)
        }

        private fun metersTolat(meters: Double): Double {
            return Math.toDegrees(meters / RADIUS_OF_EARTH)
        }

        fun latToMeters(lat: Double): Double {
            return Math.toRadians(lat) * RADIUS_OF_EARTH
        }

        /**
         * Extrapolate latitude/longitude given a heading and distance thanks to
         * http://www.movable-type.co.uk/scripts/latlong.html
         *
         * @param origin   Point of origin
         * @param bearing  bearing to navigate
         * @param distance distance to be added
         * @return New point with the added distance
         */
        @JvmStatic
        fun newCoordFromBearingAndDistance(origin: LatLong, bearing: Double, distance: Double): LatLong {
            return newCoordFromBearingAndDistance(origin.latitude, origin.longitude, bearing, distance)
        }

        /**
         * Extrapolate latitude/longitude given a heading and distance thanks to
         * http://www.movable-type.co.uk/scripts/latlong.html
         *
         * @param lat   latitude
         * @param lon   longitude
         * @param bearing  bearing to navigate
         * @param distance distance to be added
         * @return New point with the added distance
         */
        @JvmStatic
        fun newCoordFromBearingAndDistance(lat: Double, lon: Double, bearing: Double, distance: Double): LatLong {
            val lat1 = Math.toRadians(lat)
            val lon1 = Math.toRadians(lon)
            val brng = Math.toRadians(bearing)
            val dr = distance / RADIUS_OF_EARTH
            val lat2 = Math.asin(Math.sin(lat1) * Math.cos(dr) + (Math.cos(lat1) * Math.sin(dr)
                    * Math.cos(brng)))
            val lon2 = (lon1
                    + Math.atan2(Math.sin(brng) * Math.sin(dr) * Math.cos(lat1),
                    Math.cos(dr) - Math.sin(lat1) * Math.sin(lat2)))
            return LatLong(Math.toDegrees(lat2), Math.toDegrees(lon2))
        }

        /**
         * Offset a coordinate by a local distance
         *
         * @param origin  location in WGS84
         * @param xMeters Offset distance in the east direction
         * @param yMeters Offset distance in the north direction
         * @return new coordinate with the offset
         */
        @JvmStatic
        fun moveCoordinate(origin: LatLong, xMeters: Double, yMeters: Double): LatLong {
            val lon = origin.longitude
            val lat = origin.latitude
            val lon1 = Math.toRadians(lon)
            val lat1 = Math.toRadians(lat)
            val lon2 = lon1 + Math.toRadians(metersTolat(xMeters))
            val lat2 = lat1 + Math.toRadians(metersTolat(yMeters))
            return LatLong(Math.toDegrees(lat2), Math.toDegrees(lon2))
        }

        /**
         * Calculates the arc between two points
         * http://en.wikipedia.org/wiki/Haversine_formula
         *
         * @return the arc in degrees
         */
        fun getArcInRadians(from: LatLong, to: LatLong): Double {
            val latitudeArc = Math.toRadians(from.latitude - to.latitude)
            val longitudeArc = Math.toRadians(from.longitude - to.longitude)
            var latitudeH = Math.sin(latitudeArc * 0.5)
            latitudeH *= latitudeH
            var lontitudeH = Math.sin(longitudeArc * 0.5)
            lontitudeH *= lontitudeH
            val tmp = (Math.cos(Math.toRadians(from.latitude))
                    * Math.cos(Math.toRadians(to.latitude)))
            return Math.toDegrees(2.0 * Math.asin(Math.sqrt(latitudeH + tmp * lontitudeH)))
        }

        /**
         * Computes the distance between two coordinates
         *
         * @return distance in meters
         */
        @JvmStatic
        fun getDistance(from: LatLong, to: LatLong): Double {
            return RADIUS_OF_EARTH * Math.toRadians(getArcInRadians(from, to))
        }

        /**
         * Computes the distance between two coordinates taking in account the
         * height difference
         *
         * @return distance in meters
         */
        @JvmStatic
        fun get3DDistance(end: LatLongAlt, start: LatLongAlt): Double {
            val horizontalDistance = getDistance(end, start)
            val altitudeDiff = Math.abs(end.altitude - start.altitude)
            return hypot(horizontalDistance, altitudeDiff)
        }

        /**
         * Computes the heading between two coordinates
         *
         * @return heading in degrees
         */
        @JvmStatic
        fun getHeadingFromCoordinates(fromLoc: LatLong, toLoc: LatLong): Double {
            val fLat = Math.toRadians(fromLoc.latitude)
            val fLng = Math.toRadians(fromLoc.longitude)
            val tLat = Math.toRadians(toLoc.latitude)
            val tLng = Math.toRadians(toLoc.longitude)
            val degree = Math.toDegrees(Math.atan2(
                    Math.sin(tLng - fLng) * Math.cos(tLat),
                    Math.cos(fLat) * Math.sin(tLat) - (Math.sin(fLat) * Math.cos(tLat)
                            * Math.cos(tLng - fLng))))
            return warpToPositiveAngle(degree)
        }

        fun warpToPositiveAngle(degree: Double): Double {
            return if (degree >= 0) {
                degree
            } else {
                360 + degree
            }
        }

        /**
         * Copied from android-map-utils (licensed under Apache v2)
         * com.google.maps.android.SphericalUtil.java
         *
         * @return area in m�
         */
        @JvmStatic
        fun getArea(poly: Polygon): Area {
            val path = poly.points
            val size = path.size
            if (size < 3) {
                return Area(0.0)
            }
            var total = 0.0
            val prev = path[size - 1]
            var prevTanLat = Math.tan((Math.PI / 2 - Math.toRadians(prev.latitude)) / 2)
            var prevLng = Math.toRadians(prev.longitude)
            // For each edge, accumulate the signed area of the triangle formed by
            // the North Pole
            // and that edge ("polar triangle").
            for (point in path) {
                val tanLat = Math.tan((Math.PI / 2 - Math.toRadians(point.latitude)) / 2)
                val lng = Math.toRadians(point.longitude)
                total += polarTriangleArea(tanLat, lng, prevTanLat, prevLng)
                prevTanLat = tanLat
                prevLng = lng
            }
            return Area(Math.abs(total * (RADIUS_OF_EARTH * RADIUS_OF_EARTH)))
        }

        /**
         * Copied from android-map-utils (licensed under Apache v2)
         * com.google.maps.android.SphericalUtil.java
         *
         *
         * Returns the signed area of a triangle which has North Pole as a vertex.
         * Formula derived from
         * "Area of a spherical triangle given two edges and the included angle" as
         * per "Spherical Trigonometry" by Todhunter, page 71, section 103, point 2.
         * See http://books.google.com/books?id=3uBHAAAAIAAJ&pg=PA71 The arguments
         * named "tan" are tan((pi/2 - latitude)/2).
         */
        private fun polarTriangleArea(tan1: Double, lng1: Double, tan2: Double, lng2: Double): Double {
            val deltaLng = lng1 - lng2
            val t = tan1 * tan2
            return 2 * Math.atan2(t * Math.sin(deltaLng), 1 + t * Math.cos(deltaLng))
        }

        fun pointAlongTheLine(start: LatLong, end: LatLong, distance: Int): LatLong {
            return newCoordFromBearingAndDistance(start, getHeadingFromCoordinates(start, end), distance.toDouble())
        }
    }
}

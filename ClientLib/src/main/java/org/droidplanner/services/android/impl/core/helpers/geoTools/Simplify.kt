package org.droidplanner.services.android.impl.core.helpers.geoTools

import com.o3dr.services.android.lib.coordinate.LatLong
import org.droidplanner.services.android.impl.core.helpers.geoTools.PointTools.pointToLineDistance
import java.util.*

/**
 * Based on the Ramer–Douglas–Peucker algorithm algorithm
 * http://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
 */
object Simplify {
    fun simplify(list: List<LatLong?>, tolerance: Double): ArrayList<LatLong?> {
        var index = 0
        var dmax = 0.0
        val lastIndex = list.size - 1

        // Find the point with the maximum distance
        for (i in 1 until lastIndex) {
            val d = pointToLineDistance(list[0]!!, list[lastIndex]!!, list[i]!!)
            if (d > dmax) {
                index = i
                dmax = d
            }
        }

        // If max distance is greater than epsilon, recursively simplify
        val ResultList = ArrayList<LatLong?>()
        if (dmax > tolerance) {
            // Recursive call
            val recResults1 = simplify(list.subList(0, index + 1), tolerance)
            val recResults2 = simplify(list.subList(index, lastIndex + 1), tolerance)

            // Build the result list
            recResults1.removeAt(recResults1.size - 1)
            ResultList.addAll(recResults1)
            ResultList.addAll(recResults2)
        } else {
            ResultList.add(list[0])
            ResultList.add(list[lastIndex])
        }

        // Return the result
        return ResultList
    }
}

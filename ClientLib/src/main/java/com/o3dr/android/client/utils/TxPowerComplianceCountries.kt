package com.o3dr.android.client.utils

import com.o3dr.android.client.utils.TxPowerComplianceCountries

/**
 * Created by chavi on 1/27/16.
 */
enum class TxPowerComplianceCountries(val prettyName: String) {
    AU("Australia"),
    FR("European Union"),
    JP("Japan"),
    US("United States")
    ;

    companion object {
        @JvmStatic
        val defaultCountry: TxPowerComplianceCountries
            get() = US
        val defaultEUCountry: TxPowerComplianceCountries
            get() = FR
    }
}

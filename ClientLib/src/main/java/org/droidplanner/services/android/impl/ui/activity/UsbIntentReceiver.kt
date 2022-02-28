package org.droidplanner.services.android.impl.ui.activity

import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import com.o3dr.android.client.R
import android.content.Intent

/**
 * Created by fhuya on 11/12/14.
 */
class UsbIntentReceiver : AppCompatActivity() {
    public override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_usb_intent_receiver)
        handleIntent(intent)
    }

    public override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        handleIntent(intent)
    }

    private fun handleIntent(intent: Intent) {
        finish()
    }
}

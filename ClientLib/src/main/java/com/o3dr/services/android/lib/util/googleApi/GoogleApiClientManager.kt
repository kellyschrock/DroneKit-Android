package com.o3dr.services.android.lib.util.googleApi

import android.content.Context
import com.google.android.gms.common.api.Api.ApiOptions.NotRequiredOptions
import com.google.android.gms.common.api.GoogleApiClient.ConnectionCallbacks
import com.google.android.gms.common.api.GoogleApiClient.OnConnectionFailedListener
import android.os.HandlerThread
import com.google.android.gms.common.api.GoogleApiClient
import com.google.android.gms.common.GooglePlayServicesUtil
import android.os.Bundle
import android.os.Handler
import android.util.Log
import com.google.android.gms.common.ConnectionResult
import com.google.android.gms.common.api.Api
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Handles the lifecycle for the google api client. Also takes care of running submitted tasks
 * when the google api client is connected.
 */
class GoogleApiClientManager(
        /**
         * Application context.
         */
        private val context: Context,
        /**
         * This handler is in charge of running google api client tasks on the calling thread.
         */
        private val mainHandler: Handler,
        apis: Array<Api<out NotRequiredOptions?>?>) : ConnectionCallbacks, OnConnectionFailedListener {
    interface ManagerListener {
        fun onGoogleApiConnectionError(result: ConnectionResult?)
        fun onUnavailableGooglePlayServices(status: Int)
        fun onManagerStarted()
        fun onManagerStopped()
    }

    /**
     * Manager background thread used to run the submitted google api client tasks.
     */
    private val mDriverRunnable = Runnable {
        try {
            while (isStarted.get()) {
                if (!googleApiClient.isConnected) {
                    stop()
                    continue
                }
                val task = taskQueue.take() ?: continue
                if (task.runOnBackgroundThread) {
                    mBgHandler!!.post(task)
                } else {
                    mainHandler.post(task)
                }
            }
        } catch (e: InterruptedException) {
            Log.v(TAG, e.message, e)
        }
    }
    private val stopTask: GoogleApiClientTask = object : GoogleApiClientTask() {
        override fun doRun() {
            stop()
        }
    }
    private val isStarted = AtomicBoolean(false)
    private var mDriverThread: Thread? = null

    /**
     * This handler is in charge of running google api client tasks on the background thread.
     */
    private var mBgHandler: Handler? = null
    private var mBgHandlerThread: HandlerThread? = null

    /**
     * Handle to the google api client.
     */
    private val googleApiClient: GoogleApiClient
    private var listener: ManagerListener? = null

    /**
     * Holds tasks that needs to be run using the google api client.
     * A background thread will be blocking on this queue until new tasks are inserted. In which
     * case, it will retrieve the new task, and process it.
     */
    private val taskQueue = LinkedBlockingQueue<GoogleApiClientTask>()
    fun setManagerListener(listener: ManagerListener?) {
        this.listener = listener
    }

    private fun destroyBgHandler() {
        if (mBgHandlerThread != null && mBgHandlerThread!!.isAlive) {
            mBgHandlerThread!!.quit()
            mBgHandlerThread!!.interrupt()
            mBgHandlerThread = null
        }
        mBgHandler = null
    }

    private fun destroyDriverThread() {
        if (mDriverThread != null && mDriverThread!!.isAlive) {
            mDriverThread!!.interrupt()
            mDriverThread = null
        }
    }

    private fun initializeBgHandler() {
        if (mBgHandlerThread == null || mBgHandlerThread!!.isInterrupted) {
            mBgHandlerThread = HandlerThread("GAC Manager Background Thread")
            mBgHandlerThread!!.start()
            mBgHandler = null
        }
        if (mBgHandler == null) {
            mBgHandler = Handler(mBgHandlerThread!!.looper)
        }
    }

    private fun initializeDriverThread() {
        if (mDriverThread == null || mDriverThread!!.isInterrupted) {
            mDriverThread = Thread(mDriverRunnable, "GAC Manager Driver Thread")
            mDriverThread!!.start()
        }
    }

    /**
     * Adds a task to the google api client manager tasks queue. This task will be scheduled to
     * run on the calling thread.
     *
     * @param task task making use of the google api client.
     * @return true if the task was successfully added to the queue.
     * @throws IllegalStateException is the start() method was not called.
     */
    fun addTask(task: GoogleApiClientTask): Boolean {
        if (!isStarted()) {
            Log.d(TAG, "GoogleApiClientManager is not started.")
            return false
        }

        task.googleApiClient = googleApiClient
        task.taskQueue = taskQueue
        task.runOnBackgroundThread = false
        return taskQueue.offer(task)
    }

    /**
     * Adds a task to the google api client manager tasks queue. This task will be scheduled to
     * run on a background thread.
     *
     * @param task task making use of the google api client.
     * @return true if the task was successfully added to the queue.
     * @throws IllegalStateException is the start() method was not called.
     */
    fun addTaskToBackground(task: GoogleApiClientTask): Boolean {
        if (!isStarted()) {
            Log.d(TAG, "GoogleApiClientManager is not started.")
            return false
        }

        task.googleApiClient = googleApiClient
        task.taskQueue = taskQueue
        task.runOnBackgroundThread = true

        return taskQueue.offer(task)
    }

    /**
     * @return true the google api client manager was started.
     */
    private fun isStarted(): Boolean {
        return isStarted.get()
    }

    /**
     * Activates the google api client manager.
     */
    fun start() {
        //Check if google play services is available.
        val playStatus = GooglePlayServicesUtil.isGooglePlayServicesAvailable(context)
        val isValid = playStatus == ConnectionResult.SUCCESS
        if (isValid) {
            //Clear the queue
            taskQueue.clear()

            //Toggle the started flag
            isStarted.set(true)
            if (googleApiClient.isConnected) {
                onConnected(null)
            } else if (!googleApiClient.isConnecting) {
                //Connect to the google api.
                googleApiClient.connect()
            }
        } else {
            Log.e(TAG, "Google Play Services is unavailable.")
            if (listener != null) listener!!.onUnavailableGooglePlayServices(playStatus)
        }
    }
    //    private boolean isGooglePlayServicesValid(){
    //        // Check for the google play services is available
    //
    //        if(!isValid){
    //            PendingIntent errorPI = GooglePlayServicesUtil.getErrorPendingIntent(playStatus, mContext, 0);
    //            if(errorPI != null){
    //                try {
    //                    errorPI.send();
    //                } catch (PendingIntent.CanceledException e) {
    //                    Log.e(TAG, "Seems the pending intent was cancelled.", e);
    //                }
    //            }
    //        }
    //
    //        return isValid;
    //    }
    /**
     * Release the resources used by this manager.
     * After calling this method, start() needs to be called again to use that manager again.
     */
    private fun stop() {
        isStarted.set(false)
        destroyDriverThread()
        destroyBgHandler()
        taskQueue.clear()
        if (googleApiClient.isConnected || googleApiClient.isConnecting) {
            googleApiClient.disconnect()
        }
        if (listener != null) listener!!.onManagerStopped()
    }

    fun stopSafely() {
        addTask(stopTask)
    }

    override fun onConnected(bundle: Bundle?) {
        initializeBgHandler()
        initializeDriverThread()
        if (listener != null) listener!!.onManagerStarted()
    }

    override fun onConnectionSuspended(i: Int) {}
    override fun onConnectionFailed(connectionResult: ConnectionResult) {
        if (listener != null) listener!!.onGoogleApiConnectionError(connectionResult)
        stop()
    }

    /**
     * Type for the google api client tasks.
     */
    abstract class GoogleApiClientTask : Runnable {
        /**
         * If true, this task will be scheduled to run on a background thread.
         * Otherwise, it will run on the calling thread.
         */
        var runOnBackgroundThread = false
        var googleApiClient: GoogleApiClient? = null
        var taskQueue: LinkedBlockingQueue<GoogleApiClientTask>? = null
        override fun run() {
            if (!googleApiClient!!.isConnected) {
                //Add the task back to the queue.
                taskQueue!!.offer(this)
                return
            }

            //Run the task
            doRun()
        }

        protected abstract fun doRun()
    }

    companion object {
        private val TAG = GoogleApiClientManager::class.java.simpleName
    }

    init {
        val apiBuilder = GoogleApiClient.Builder(context)
        for (api in apis) {
            apiBuilder.addApi(api!!)
        }
        googleApiClient = apiBuilder
                .addConnectionCallbacks(this)
                .addOnConnectionFailedListener(this)
                .build()
    }
}

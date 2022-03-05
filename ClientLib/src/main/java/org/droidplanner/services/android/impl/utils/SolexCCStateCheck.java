package org.droidplanner.services.android.impl.utils;

import android.os.Handler;
import android.util.Log;

import com.o3dr.services.android.lib.drone.property.SolexCCState;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.URL;

import timber.log.Timber;

public class SolexCCStateCheck {
    private static final String TAG = SolexCCStateCheck.class.getSimpleName();

    public interface Listener {
        void onSolexCCState(SolexCCState state);
    }

    public static void check(final String ipAddress, final int port, final Listener listener) {
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    final String path = String.format("http://%s:%d/client/myip", ipAddress, port);
                    final URL url = new URL(path);
                    final byte[] buf = new byte[2048];
                    final InputStream input = url.openStream();
                    final OutputStream out = new ByteArrayOutputStream();
                    try {
                        for(int read = input.read(buf); read != -1; read = input.read(buf)) {
                            out.write(buf, 0, read);
                        }
                    } finally {
                        out.flush();
                        out.close();
                        input.close();
                    }

                    final String localIp = out.toString();
                    Timber.d("destIp=%s localIp=%s", ipAddress, localIp);

                    final SolexCCState state = new SolexCCState(ipAddress);
                    listener.onSolexCCState(state);

                } catch(Throwable ex) {
                    Log.e(TAG, ex.getMessage(), ex);
                    listener.onSolexCCState(null);
                }
            }
        }).start();
    }
}

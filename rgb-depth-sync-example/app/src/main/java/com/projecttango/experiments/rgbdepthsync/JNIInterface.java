/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.projecttango.experiments.rgbdepthsync;

import android.app.Activity;
import android.os.IBinder;
import android.util.Log;

/**
 * Interfaces between C and Java.
 */
public class JNIInterface {
    static {
        if (TangoInitializationHelper.loadTangoSharedLibrary() ==
                TangoInitializationHelper.ARCH_ERROR) {
            Log.e("TangoJNINative", "ERROR! Unable to load libtango_client_api.so!");
                }
      System.loadLibrary("rgb_depth_sync_example");
    }

    public static native int tangoInitialize(IBinder binder);

    public static native int tangoSetupConfig();

    public static native int tangoConnect();

    public static native int tangoConnectCallbacks();

    public static native int tangoSetIntrinsicsAndExtrinsics();

    public static native void tangoDisconnect();

    public static native void initializeGLContent();

    public static native void freeGLContent();

    public static native void setViewPort(int width, int height);

    public static native void render();
    public static native void writeCurrentData();
    public static native void startCapture(String filename);
    public static native void stopCapture();

    public static native String getAdfList();
    public static native void setAdf(String adf);

    public static native void setDepthAlphaValue(float alpha);

    public static native void setGPUUpsample(boolean on);
}

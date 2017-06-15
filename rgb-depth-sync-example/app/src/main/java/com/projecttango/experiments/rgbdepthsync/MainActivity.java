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
import android.app.DialogFragment;
import android.app.Dialog;
import android.content.Context;
import android.content.ComponentName;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.ServiceConnection;
import android.database.Cursor;
import android.graphics.Point;
import android.media.MediaScannerConnection;
import android.net.Uri;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.Environment;
import android.os.IBinder;
import android.util.Log;
import android.view.Display;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.SeekBar;
import android.widget.Toast;

import java.io.File;

/**
 * Activity that load up the main screen of the app, this is the launcher activity.
 */
public class MainActivity extends Activity implements FilenameSelectDialog.FilenameSelectDialogListener, AdfSelectDialog.AdfSelectDialogListener {
    /// The input argument is invalid.
    private static final int  TANGO_INVALID = -2;
    /// This error code denotes some sort of hard error occurred.
    private static final int  TANGO_ERROR = -1;
    /// This code indicates success.
    private static final int  TANGO_SUCCESS = 0;

    // Key string for requesting and checking Motion Tracking permission.
    private static final String MOTION_TRACKING_PERMISSION =
            "MOTION_TRACKING_PERMISSION";
    private static final String AREA_LEARNING_PERMISSION =
            "ADF_LOAD_SAVE_PERMISSION";
    // Permission request action.
    private static final String REQUEST_PERMISSION_ACTION =
            "android.intent.action.REQUEST_TANGO_PERMISSION";

    private GLSurfaceView mGLView;

    private SeekBar mDepthOverlaySeekbar;
    private CheckBox mdebugOverlayCheckbox;
    private Button mStartCaptureButton;
    private CheckBox mGPUUpsampleCheckbox;

    private boolean mIsConnectedService = false;
  // Tango Service connection.
  ServiceConnection mTangoServiceConnection = new ServiceConnection() {
      public void onServiceConnected(ComponentName name, IBinder service) {
          Log.w(TAG, "Called onServiceConnected");
        JNIInterface.tangoInitialize(service);
          int ret = JNIInterface.tangoSetupConfig();
          if (ret != TANGO_SUCCESS) {
              Log.e(TAG, "Failed to set config with code: "  + ret);
              finish();
          }
          Log.w(TAG, "Done config setup");

          ret = JNIInterface.tangoConnectCallbacks();
          if (ret != TANGO_SUCCESS) {
              Log.e(TAG, "Failed to set connect cbs with code: "  + ret);
              finish();
          }
          Log.w(TAG, "Done callbacks");

          ret = JNIInterface.tangoConnect();
          if (ret != TANGO_SUCCESS) {
              Log.e(TAG, "Failed to set connect service with code: "  + ret);
              finish();
          }

          Log.w(TAG, "Done connect");
          ret = JNIInterface.tangoSetIntrinsicsAndExtrinsics();
          if (ret != TANGO_SUCCESS) {
              Log.e(TAG, "Failed to extrinsics and intrinsics code: "  + ret);
              finish();
          }
          Log.w(TAG, "Done intrinsics");

          mIsConnectedService = true;
          Log.w(TAG, "Successful init");
        //setAndroidOrientation();
      }

      public void onServiceDisconnected(ComponentName name) {
        // Handle this if you need to gracefully shutdown/retry
        // in the event that Tango itself crashes/gets upgraded while running.
      }
    };

    private static final String TAG = "RGBDepthSync";

    private Thread mWriterThread;

    private boolean capturing;
    private String savefilename;

    private class DepthOverlaySeekbarListener implements SeekBar.OnSeekBarChangeListener {
        @Override
        public void onProgressChanged(SeekBar seekBar, int progress,
                boolean fromUser) {
            JNIInterface.setDepthAlphaValue((float) progress / (float) seekBar.getMax());
        }

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {}

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {}
    }

    private class DebugOverlayCheckboxListener implements CheckBox.OnCheckedChangeListener {
        @Override
        public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
            if (buttonView == mdebugOverlayCheckbox) {
                if (isChecked) {
                    float progress = mDepthOverlaySeekbar.getProgress();
                    float max = mDepthOverlaySeekbar.getMax();
                    JNIInterface.setDepthAlphaValue(progress / max);
                    mDepthOverlaySeekbar.setVisibility(View.VISIBLE);
                } else {
                    JNIInterface.setDepthAlphaValue(0.0f);
                    mDepthOverlaySeekbar.setVisibility(View.GONE);
                }
            }
        }
    }

    // FilenameSelectDialogListener methods
    public void onPositiveClick(String s) {
        mStartCaptureButton.setText(R.string.button_stop_capture);
        savefilename = s;
        String path = (new File(getExternalFilesDir(null), savefilename)).getAbsolutePath();
        Log.w(TAG, path);
        JNIInterface.startCapture(path);
        capturing = true;
        mWriterThread = new Thread(new Runnable() {
            public void run() {
                while(capturing) {
                    JNIInterface.writeCurrentData();
                }
            }
        });
        mWriterThread.start();
    }
    public void onNegativeClick(String s) {
    }

    // AdfSelectDialogListener methods
    public void onAdfSelected(String s) {
        JNIInterface.setAdf(s);
        // FIXME: Disable start capture until localized
        JNIInterface.tangoDisconnect();
        mIsConnectedService = false;
        int ret = JNIInterface.tangoSetupConfig();
        if (ret != TANGO_SUCCESS) {
            Log.e(TAG, "Failed to set config with code: "  + ret);
            finish();
        }

        ret = JNIInterface.tangoConnectCallbacks();
        if (ret != TANGO_SUCCESS) {
            Log.e(TAG, "Failed to set connect cbs with code: "  + ret);
            finish();
        }

        ret = JNIInterface.tangoConnect();
        if (ret != TANGO_SUCCESS) {
            Log.e(TAG, "Failed to set connect service with code: "  + ret);
            finish();
        }
        mIsConnectedService = true;
    }
    public void onAdfCancel() {

    }
    public void stopCapture() {
        mStartCaptureButton.setText(R.string.button_start_capture);
        JNIInterface.stopCapture();
        capturing = false;
    }

    private class GPUUpsampleListener implements CheckBox.OnCheckedChangeListener {
        @Override
        public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
            JNIInterface.setGPUUpsample(isChecked);
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        Display display = getWindowManager().getDefaultDisplay();
        Point size = new Point();
        display.getSize(size);

        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);

        setContentView(R.layout.activity_main);

        mDepthOverlaySeekbar = (SeekBar) findViewById(R.id.depth_overlay_alpha_seekbar);
        mDepthOverlaySeekbar.setOnSeekBarChangeListener(new DepthOverlaySeekbarListener());
        mDepthOverlaySeekbar.setVisibility(View.GONE);

        mdebugOverlayCheckbox = (CheckBox) findViewById(R.id.debug_overlay_checkbox);
        mdebugOverlayCheckbox.setOnCheckedChangeListener(new DebugOverlayCheckboxListener());

        mGPUUpsampleCheckbox = (CheckBox) findViewById(R.id.gpu_upsample_checkbox);
        mGPUUpsampleCheckbox.setOnCheckedChangeListener(new GPUUpsampleListener());

        mStartCaptureButton = (Button) findViewById(R.id.startCaptureButton);
        mStartCaptureButton.setOnClickListener(new Button.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (capturing) {
                    stopCapture();
                }
                else {
                    DialogFragment saveDialog = new FilenameSelectDialog();
                    saveDialog.show(getFragmentManager(), "saveDialog");
                }
            }
        });

        Button mOpenAdfButton = (Button) findViewById(R.id.openAdfButton);
        mOpenAdfButton.setOnClickListener(new Button.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (capturing) {
                    stopCapture();
                }
                String s = JNIInterface.getAdfList();
                if (s.length() == 0) {
                    (new DialogFragment() {
                        @Override
                        public Dialog onCreateDialog(Bundle savedInstanceState) {
                            android.app.AlertDialog.Builder builder = new android.app.AlertDialog.Builder(getActivity());
                            builder.setMessage("No ADF files found")
                                .setPositiveButton("OK", new DialogInterface.OnClickListener() {
                                    @Override
                                    public void onClick(DialogInterface dialogInterface, int i) {
                                    }
                                });
                            return builder.create();
                        }
                    }).show(getFragmentManager(), "errorNoAdf");
                } else {
                    AdfSelectDialog selectAdfDialog = new AdfSelectDialog();
                    selectAdfDialog.setAdfstring(s);
                    selectAdfDialog.show(getFragmentManager(), "selectAdf");
                }
            }
        });

        // OpenGL view where all of the graphics are drawn
        mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);

        // Configure OpenGL renderer
        mGLView.setEGLContextClientVersion(2);
        GLSurfaceRenderer mRenderer = new GLSurfaceRenderer(this);
        mGLView.setRenderer(mRenderer);
    }

    @Override
    protected void onResume() {
        // We moved most of the onResume lifecycle calls to the surfaceCreated,
        // surfaceCreated will be called after the GLSurface is created.
        super.onResume();

        // Though we're going to use Tango's C interface so that we have more
        // low level control of our graphics, we can still use the Java API to
        // check that we have the correct permissions.
        if (!hasPermission(this, MOTION_TRACKING_PERMISSION)) {
            getPermission(MOTION_TRACKING_PERMISSION);
        } else {
            if (!hasPermission(this, AREA_LEARNING_PERMISSION)) {
                getPermission(AREA_LEARNING_PERMISSION);
            } else {
                mGLView.onResume();
            }
        }
        //TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
    }

    @Override
    protected void onPause() {
        super.onPause();
        mGLView.onPause();
        if (mIsConnectedService) {
            if (capturing) stopCapture();
            unbindService(mTangoServiceConnection);
        }
        if (savefilename != null && savefilename.length() > 0) {
            String[] filenames = new String[]{
                    (new File(getExternalFilesDir(null), savefilename)).getAbsolutePath(),
                    (new File(getExternalFilesDir(null), savefilename + ".xforms")).getAbsolutePath(),
                    (new File(getExternalFilesDir(null), savefilename + ".pts")).getAbsolutePath()
            };
            sendBroadcast(new Intent(Intent.ACTION_MEDIA_SCANNER_SCAN_FILE, Uri.parse(filenames[0])));
            sendBroadcast(new Intent(Intent.ACTION_MEDIA_SCANNER_SCAN_FILE, Uri.parse(filenames[1])));
            MediaScannerConnection.scanFile(MainActivity.this, filenames, null, null);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    public void surfaceCreated() {
        JNIInterface.initializeGLContent();
        TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
    }

    @Override
    protected void onActivityResult (int requestCode, int resultCode, Intent data) {
        if (requestCode == 0) {
            if (resultCode == RESULT_CANCELED) {
                mIsConnectedService = false;
                finish();
            }
        }
    }

    public boolean hasPermission(Context context, String permissionType){
        Uri uri = Uri.parse("content://com.google.atap.tango.PermissionStatusProvider/" +
                permissionType);
        Cursor cursor = context.getContentResolver().query(uri, null, null, null, null);
        return cursor != null;
    }

    private void getPermission(String permissionType) {
        Intent intent = new Intent();
        intent.setAction(REQUEST_PERMISSION_ACTION);
        intent.putExtra("PERMISSIONTYPE", permissionType);

        // After the permission activity is dismissed, we will receive a callback
        // function onActivityResult() with user's result.
        startActivityForResult(intent, 0);
    }
}

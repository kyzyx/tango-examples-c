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

#ifndef RGB_DEPTH_SYNC_APPLICATION_H_
#define RGB_DEPTH_SYNC_APPLICATION_H_

#include <jni.h>

#include <tango_client_api.h>
#include <rgb-depth-sync/color_image.h>
#include <rgb-depth-sync/depth_image.h>
#include <rgb-depth-sync/scene.h>
#include <rgb-depth-sync/util.h>
#include <tango-gl/util.h>

namespace rgb_depth_sync {

// This thread safe class is the main application for Synchronization.
// It can be instantiated in the JNI layer and use to pass information back and
// forth between Java. The class also manages the application's lifecycle and
// interaction with the Tango service. Primarily, this involves registering for
// callbacks and passing on the necessary information to stored objects.
class SynchronizationApplication {
 public:
  SynchronizationApplication();
  ~SynchronizationApplication();

  // Tango lifecycle calls
  int TangoInitialize(JNIEnv* env, jobject caller_activity);
  int TangoConnect();
  int TangoConnectCallbacks();
  int TangoSetupConfig();
  void TangoDisconnect();
  int TangoSetIntrinsicsAndExtrinsics();

  // OpenGL functions
  void InitializeGLContent();
  void SetViewPort(int width, int height);
  void FreeGLContent();
  void SetDepthAlphaValue(float alpha);

  // Callback functions
  void OnXYZijAvailable(const TangoXYZij* xyz_ij);
  void OnColorFrameAvailable(const TangoImageBuffer* buffer);
  void OnPoseAvailable(const TangoPoseData* pose);

  // Data capture functions
  void startCapture(std::string filename);
  void stopCapture();
  void writeCurrentData();

  // ADF functions
  std::string getAdfList();
  void setAdf(std::string adfuuid) { uuid = adfuuid; }

  // Main Render loop.
  void Render();

 private:
  // Color image buffers
  // Data is assumed to be in YUV format (NV21)
  std::vector<GLubyte> rendercallback_yuv_buffer_;
  std::vector<GLubyte> outputcallback_yuv_buffer_;
  std::vector<GLubyte> render_yuv_buffer_;
  std::vector<GLubyte> output_yuv_buffer_;
  std::vector<GLubyte> rendershared_yuv_buffer_;
  std::vector<GLubyte> outputshared_yuv_buffer_;

  double outputcolor_timestamp_;
  std::vector<double> outputcolor_timestamps;
  double rendercolor_timestamp_;
  std::mutex outputcolor_mutex_;
  std::mutex rendercolor_mutex_;
  bool outputyuv_swap_signal;
  bool renderyuv_swap_signal;

  // Depth image buffers
  std::vector<float> rendercallback_point_cloud_buffer_;
  std::vector<float> outputcallback_point_cloud_buffer_;
  std::vector<float> rendershared_point_cloud_buffer_;
  std::vector<float> outputshared_point_cloud_buffer_;
  std::vector<float> render_point_cloud_buffer_;
  std::vector<float> output_point_cloud_buffer_;
  std::vector<uint32_t> callback_pointindices_;
  std::vector<uint32_t> shared_pointindices_;
  std::vector<uint32_t> output_pointindices_;

  int depthw, depthh;

  double outputdepth_timestamp_;
  std::vector<double> outputdepth_timestamps;
  double renderdepth_timestamp_;
  std::mutex outputpoint_cloud_mutex_;
  std::mutex renderpoint_cloud_mutex_;
  bool outputdepth_swap_signal;
  bool renderdepth_swap_signal;

  // Data capture variables
  std::mutex write_mutex_;
  FILE* datadump;
  FILE* depthdatadump;
  FILE* imageposes;
  bool capture;

  // Area learning variables
  std::string uuid;

  // Pose variables
  TangoCameraIntrinsics color_camera_intrinsics;
  bool tracking;
  bool localized;
  std::mutex pose_mutex_;

  // OpenGL helper classes
  ColorImage* color_image_;
  DepthImage* depth_image_;
  Scene* main_scene_;

  // Transforms
  glm::mat4 device_T_color_;
  glm::mat4 device_T_depth_;
  glm::mat4 OW_T_SS_;

  TangoConfig tango_config_;

  float screen_width_;
  float screen_height_;

};
}  // namespace rgb_depth_sync

#endif  // RGB_DEPTH_SYNC_APPLICATION_H_

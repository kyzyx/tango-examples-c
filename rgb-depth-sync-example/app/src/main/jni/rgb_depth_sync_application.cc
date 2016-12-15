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
#include <tango-gl/conversions.h>

#include <rgb-depth-sync/rgb_depth_sync_application.h>
using namespace std;

namespace rgb_depth_sync {

#pragma region callbacks
// Callback routers
void OnXYZijAvailableRouter(void* context, const TangoXYZij* xyz_ij) {
  SynchronizationApplication* app =
      static_cast<SynchronizationApplication*>(context);
  app->OnXYZijAvailable(xyz_ij);
}

void OnColorFrameAvailableRouter(
        void* context, TangoCameraId id, const TangoImageBuffer* buffer)
{
  SynchronizationApplication* app =
      static_cast<SynchronizationApplication*>(context);
  app->OnColorFrameAvailable(buffer);
}
void OnPoseAvailableRouter(
        void* context, const TangoPoseData* pose)
{
  SynchronizationApplication* app =
      static_cast<SynchronizationApplication*>(context);
  app->OnPoseAvailable(pose);
}

// Callbacks
void SynchronizationApplication::OnXYZijAvailable(const TangoXYZij* xyz_ij) {
  // We'll just update the point cloud associated with our depth image.
  size_t point_cloud_size = xyz_ij->xyz_count * 3;
  if (datadump != NULL) {
      outputcallback_point_cloud_buffer_.resize(point_cloud_size);
      int w = xyz_ij->ij_cols;
      int h = xyz_ij->ij_rows;
      callback_pointindices_.resize(w*h);
      std::copy(xyz_ij->xyz[0], xyz_ij->xyz[0] + point_cloud_size,
              outputcallback_point_cloud_buffer_.begin());
      std::copy(xyz_ij->ij, xyz_ij->ij + w*h,
              callback_pointindices_.begin());
    {
        std::lock_guard<std::mutex> lock(outputpoint_cloud_mutex_);
        depthw = w;
        depthh = h;
        outputdepth_timestamp_ = xyz_ij->timestamp;
        outputcallback_point_cloud_buffer_.swap(outputshared_point_cloud_buffer_);
        callback_pointindices_.swap(shared_pointindices_);
        outputdepth_swap_signal = true;
    }
  }
  rendercallback_point_cloud_buffer_.resize(point_cloud_size);
  std::copy(xyz_ij->xyz[0], xyz_ij->xyz[0] + point_cloud_size,
            rendercallback_point_cloud_buffer_.begin());
  {
    std::lock_guard<std::mutex> lock(renderpoint_cloud_mutex_);
    renderdepth_timestamp_ = xyz_ij->timestamp;
    rendercallback_point_cloud_buffer_.swap(rendershared_point_cloud_buffer_);
    renderdepth_swap_signal = true;
  }
}

void SynchronizationApplication::OnColorFrameAvailable(const TangoImageBuffer* buffer) {
    int w = buffer->width;
    int h = buffer->height;
    int fmt;
    if (buffer->format == TANGO_HAL_PIXEL_FORMAT_YV12) fmt = 3;
    else if (buffer->format == TANGO_HAL_PIXEL_FORMAT_RGBA_8888) fmt = 1;
    else if (buffer->format == TANGO_HAL_PIXEL_FORMAT_YCrCb_420_SP) fmt = 2;
    //LOGE("%d x %d with format %d, size %d\n", w, h, fmt, buffer->stride);
    // Save images
    int bufsz = w*h*3/2;
    if (datadump != NULL) {
        std::copy(buffer->data, buffer->data + bufsz,
                outputcallback_yuv_buffer_.begin());
        {
            std::lock_guard<std::mutex> lock(outputcolor_mutex_);
            outputcallback_yuv_buffer_.swap(outputshared_yuv_buffer_);
            outputyuv_swap_signal = true;
            outputcolor_timestamp_ = buffer->timestamp;
        }
    }
    std::copy(buffer->data, buffer->data + bufsz,
            rendercallback_yuv_buffer_.begin());
    {
        std::lock_guard<std::mutex> lock(rendercolor_mutex_);
        rendercallback_yuv_buffer_.swap(rendershared_yuv_buffer_);
        renderyuv_swap_signal = true;
        rendercolor_timestamp_ = buffer->timestamp;
    }
}

void SynchronizationApplication::OnPoseAvailable(const TangoPoseData* pose) {
    if (pose->frame.base == TANGO_COORDINATE_FRAME_AREA_DESCRIPTION
            && pose->frame.target == TANGO_COORDINATE_FRAME_START_OF_SERVICE)
    {
        if (pose->status_code == TANGO_POSE_VALID) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (!localized) localized = true;
        } else {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (localized) localized = false;
        }
    }
}

#pragma endregion

#pragma region Initialization and Cleanup
SynchronizationApplication::SynchronizationApplication() :
    gpu_upsample_(false) {
  // We'll store the fixed transform between the opengl frame convention.
  // (Y-up, X-right) and tango frame convention. (Z-up, X-right).
  OW_T_SS_ = tango_gl::conversions::opengl_world_T_tango_world();
  rendercallback_yuv_buffer_.resize(1920*1080*3/2);
  outputcallback_yuv_buffer_.resize(1920*1080*3/2);
  render_yuv_buffer_.resize(1920*1080*3/2);
  output_yuv_buffer_.resize(1920*1080*3/2);
  rendershared_yuv_buffer_.resize(1920*1080*3/2);
  outputshared_yuv_buffer_.resize(1920*1080*3/2);
  tracking = false;
  localized = false;
  tango_initialized = false;
  gl_initialized = false;
}

SynchronizationApplication::~SynchronizationApplication() {}

int SynchronizationApplication::TangoInitialize(JNIEnv* env,
                                                jobject binder) {
  TangoErrorType ret = TangoService_setBinder(env, binder);
}

int SynchronizationApplication::TangoSetupConfig() {
  tango_config_ = TangoService_getConfig(TANGO_CONFIG_DEFAULT);
  if (tango_config_ == nullptr) {
    return TANGO_ERROR;
  }

  TangoErrorType ret =
      TangoConfig_setBool(tango_config_, "config_enable_depth", true);
  if (ret != TANGO_SUCCESS) {
    LOGE("Failed to enable depth.");
    return ret;
  }

  // Note that it's super important for AR applications that we enable low
  // latency imu integration so that we have pose information available as
  // quickly as possible. Without setting this flag, you'll often receive
  // invalid poses when calling GetPoseAtTime for an image.
  ret = TangoConfig_setBool(tango_config_,
                            "config_enable_low_latency_imu_integration", true);
  if (ret != TANGO_SUCCESS) {
    LOGE("Failed to enable low latency imu integration.");
    return ret;
  }

  if (uuid.length() > 0) {
      ret = TangoConfig_setString(tango_config_,
              "config_load_area_description_UUID", uuid.c_str());
      if (ret != TANGO_SUCCESS) {
          LOGE("Failed to load area description file %s.", uuid.c_str());
          return ret;
      }
  }
  return ret;
}

int SynchronizationApplication::TangoConnectCallbacks() {
  TangoErrorType ret =
      TangoService_connectOnXYZijAvailable(OnXYZijAvailableRouter);
  if (ret != TANGO_SUCCESS) return ret;
  ret = TangoService_connectOnFrameAvailable(
      TANGO_CAMERA_COLOR, this, OnColorFrameAvailableRouter);
  if (ret != TANGO_SUCCESS) return ret;
  if (uuid.length() > 0) {
      TangoCoordinateFramePair pair;
      pair.base = TANGO_COORDINATE_FRAME_AREA_DESCRIPTION;
      pair.target = TANGO_COORDINATE_FRAME_START_OF_SERVICE;
      ret = TangoService_connectOnPoseAvailable(
              1, &pair, OnPoseAvailableRouter, this);
      if (ret != TANGO_SUCCESS) return ret;
  }
  return ret;
}

int SynchronizationApplication::TangoConnect() {
  TangoErrorType ret = TangoService_connect(this, tango_config_);
  if (ret != TANGO_SUCCESS) {
    LOGE("SynchronizationApplication: Failed to connect to the Tango service.");
  } else {
      tango_initialized = true;
  }
  return ret;
}
int SynchronizationApplication::TangoSetIntrinsicsAndExtrinsics() {
  // Get the intrinsics for the color camera and pass them on to the depth
  // image. We need these to know how to project the point cloud into the color
  // camera frame.
  TangoErrorType ret = TangoService_getCameraIntrinsics(
      TANGO_CAMERA_COLOR, &color_camera_intrinsics);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "SynchronizationApplication: Failed to get the intrinsics for the color"
        "camera.");
    return ret;
  }
  depth_image_->SetCameraIntrinsics(color_camera_intrinsics);
  main_scene_->SetCameraIntrinsics(color_camera_intrinsics);

  float image_width = static_cast<float>(color_camera_intrinsics.width);
  float image_height = static_cast<float>(color_camera_intrinsics.height);
  float image_plane_ratio = image_height / image_width;
  float screen_ratio = screen_height_ / screen_width_;

  TangoPoseData pose_imu_T_device;
  TangoPoseData pose_imu_T_color;
  TangoPoseData pose_imu_T_depth;
  TangoCoordinateFramePair frame_pair;
  glm::vec3 translation;
  glm::quat rotation;

  // We need to get the extrinsic transform between the color camera and the
  // imu coordinate frames. This matrix is then used to compute the extrinsic
  // transform between color camera and device:
  // color_T_device = color_T_imu * imu_T_device;
  frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
  frame_pair.target = TANGO_COORDINATE_FRAME_DEVICE;
  ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_device);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "SynchronizationApplication: Failed to get transform between the IMU "
        "and device frames. Something is wrong with device extrinsics.");
    return ret;
  }
  glm::mat4 imu_T_device = util::GetMatrixFromPose(&pose_imu_T_device);

  frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
  frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_COLOR;
  ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_color);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "SynchronizationApplication: Failed to get transform between the IMU "
        "and color camera frames. Something is wrong with device extrinsics.");
    return ret;
  }
  glm::mat4 imu_T_color = util::GetMatrixFromPose(&pose_imu_T_color);

  frame_pair.base = TANGO_COORDINATE_FRAME_IMU;
  frame_pair.target = TANGO_COORDINATE_FRAME_CAMERA_DEPTH;
  ret = TangoService_getPoseAtTime(0.0, frame_pair, &pose_imu_T_depth);
  if (ret != TANGO_SUCCESS) {
    LOGE(
        "SynchronizationApplication: Failed to get transform between the IMU "
        "and depth camera frames. Something is wrong with device extrinsics.");
    return ret;
  }
  glm::mat4 imu_T_depth = util::GetMatrixFromPose(&pose_imu_T_depth);

  device_T_color_ = glm::inverse(imu_T_device) * imu_T_color;
  device_T_depth_ = glm::inverse(imu_T_device) * imu_T_depth;

  return ret;
}

void SynchronizationApplication::TangoDisconnect() {
  if (capture) {
      stopCapture();
  }
  TangoConfig_free(tango_config_);
  tango_config_ = nullptr;
  TangoService_disconnect();
}

void SynchronizationApplication::InitializeGLContent() {
  depth_image_ = new rgb_depth_sync::DepthImage();
  color_image_ = new rgb_depth_sync::ColorImage();
  main_scene_ = new rgb_depth_sync::Scene(color_image_, depth_image_);
  gl_initialized = true;

}

void SynchronizationApplication::SetViewPort(int width, int height) {
  screen_width_ = static_cast<float>(width);
  screen_height_ = static_cast<float>(height);
  main_scene_->SetupViewPort(width, height);
}

void SynchronizationApplication::FreeGLContent() {
  delete color_image_;
  delete depth_image_;
  delete main_scene_;
}

void SynchronizationApplication::SetDepthAlphaValue(float alpha) {
  main_scene_->SetDepthAlphaValue(alpha);
}

std::string SynchronizationApplication::getAdfList() {
  char* uuid_list = NULL;
  if (TangoService_getAreaDescriptionUUIDList(&uuid_list) != TANGO_SUCCESS) {
      LOGI("TangoService_getAreaDescriptionUUIDList");
  }
  if (uuid_list) {
      vector<string> adf_list;

      char* parsing_char;
      parsing_char = strtok(uuid_list, ",");
      while (parsing_char != NULL) {
          string s = string(parsing_char);
          adf_list.push_back(s);
          parsing_char = strtok(NULL, ",");
      }

      string ret = "";
      for (int i = 0; i < adf_list.size(); i++) {
          char* name = util::GetUUIDMetadataValue(adf_list[i].c_str(), "name");
          ret = ret + name + ":" + adf_list[i] + ";";
      }
      return ret;
  }
  return "";
}

#pragma endregion

#pragma region Data Capture methods
void SynchronizationApplication::startCapture(std::string filename) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    std::string filepath = "/sdcard/" + filename;
    datadump = fopen(filepath.c_str(), "w");
    std::string depthfilepath = filepath + ".pts";
    depthdatadump = fopen(depthfilepath.c_str(), "w");
    std::string posefilepath = "/sdcard/" + filename + ".xforms";
    imageposes = fopen(posefilepath.c_str(), "w");


    int w = color_camera_intrinsics.width;
    int h = color_camera_intrinsics.height;
    fprintf(datadump, "ImageStream\n");
    fprintf(datadump, "width %d\n", w);
    fprintf(datadump, "height %d\n", h);
    fprintf(datadump, "fx %f\n", color_camera_intrinsics.fx/w);
    fprintf(datadump, "fy %f\n", color_camera_intrinsics.fy/h);
    fprintf(datadump, "px %f\n", color_camera_intrinsics.cx/w);
    fprintf(datadump, "py %f\n", color_camera_intrinsics.cy/h);
    if (color_camera_intrinsics.calibration_type == TANGO_CALIBRATION_EQUIDISTANT) {
        fprintf(datadump, "cameraModelName equidistant\n");
    } else if (color_camera_intrinsics.calibration_type == TANGO_CALIBRATION_POLYNOMIAL_3_PARAMETERS) {
        fprintf(datadump, "cameraModelName polynomial3k\n");
    }
    for (int i = 0; i < 5; i++) {
        fprintf(datadump, "radialK%d %f\n", i, color_camera_intrinsics.distortion[i]);
    }
    fprintf(datadump, "end_header\n");

    capture = true;
}

void SynchronizationApplication::stopCapture() {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (capture) {
        capture = false;
        int ret;
        TangoPoseData pose;

        // Write sentinel values
        double tmp = -1;
        fwrite(&tmp, sizeof(double), 1, datadump);
        fwrite(&tmp, sizeof(double), 1, depthdatadump);

        // Write all poses
        for (int z = 0; z < outputcolor_timestamps.size(); z++) {
            if (uuid.length() > 0) {
                ret = util::GetGlobalPose(outputcolor_timestamps[z], &pose);
            } else {
                ret = util::GetDevicePose(outputcolor_timestamps[z], &pose);
            }
            if (ret != TANGO_SUCCESS) {
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 4; j++) {
                        float f = 0;
                        fwrite(&f, sizeof(float), 1, imageposes);
                    }
                }
            } else {
                glm::mat4 xform = util::GetMatrixFromPose(&pose) * device_T_color_;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 4; j++) {
                        float f = xform[j][i];
                        fwrite(&f, sizeof(float), 1, imageposes);
                    }
                }
            }
        }
        for (int z = 0; z < outputdepth_timestamps.size(); z++) {
            if (uuid.length() > 0) {
                ret = util::GetGlobalPose(outputdepth_timestamps[z], &pose);
            } else {
                ret = util::GetDevicePose(outputdepth_timestamps[z], &pose);
            }
            if (ret != TANGO_SUCCESS) {
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 4; j++) {
                        float f = 0;
                        fwrite(&f, sizeof(float), 1, depthdatadump);
                    }
                }
            } else {
                glm::mat4 xform = util::GetMatrixFromPose(&pose) * device_T_depth_;
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 4; j++) {
                        float f = xform[j][i];
                        fwrite(&f, sizeof(float), 1, depthdatadump);
                    }
                }
            }
        }
        fclose(datadump);
        fclose(depthdatadump);
        fclose(imageposes);
    }
}

void SynchronizationApplication::writeCurrentData() {
    std::lock_guard<std::mutex> lock(write_mutex_);
    if (!capture) return;
    // Write color data
    int w = 1920;
    int h = 1080;
    double color_timestamp, depth_timestamp;
    {
        std::lock_guard<std::mutex> lock(outputcolor_mutex_);
        if (outputyuv_swap_signal) {
            outputshared_yuv_buffer_.swap(output_yuv_buffer_);
            color_timestamp = outputcolor_timestamp_;
            outputyuv_swap_signal = false;
        }
        else return;
    }
    outputcolor_timestamps.push_back(color_timestamp);
    fwrite(&color_timestamp, sizeof(double), 1, datadump);
    fwrite(output_yuv_buffer_.data(), 1, w*h*3/2, datadump);

    // Write depth data
    int dw, dh;
    {
        std::lock_guard<std::mutex> lock(outputpoint_cloud_mutex_);
        dw = depthw;
        dh = depthh;
        if (outputdepth_swap_signal) {
            outputshared_point_cloud_buffer_.swap(output_point_cloud_buffer_);
            shared_pointindices_.swap(output_pointindices_);
            depth_timestamp = outputdepth_timestamp_;
            outputdepth_swap_signal = false;
        }
        else return;
    }
    outputdepth_timestamps.push_back(depth_timestamp);
    int sz = output_point_cloud_buffer_.size();
    fwrite(&depth_timestamp, sizeof(double), 1, depthdatadump);
    fwrite(&sz, sizeof(int), 1, depthdatadump);
    fwrite(&dw, sizeof(int), 1, depthdatadump);
    fwrite(&dh, sizeof(int), 1, depthdatadump);
    fwrite(output_point_cloud_buffer_.data(), sizeof(float), sz, depthdatadump);
    fwrite(output_pointindices_.data(), sizeof(int), dw*dh, depthdatadump);
}

#pragma endregion

void SynchronizationApplication::Render() {
    if (!tango_initialized || !gl_initialized) return;

  double color_timestamp = 0.0;
  double depth_timestamp = 0.0;
  bool new_points = false;
  {
    std::lock_guard<std::mutex> lock(renderpoint_cloud_mutex_);
    depth_timestamp = renderdepth_timestamp_;
    if (renderdepth_swap_signal) {
      rendershared_point_cloud_buffer_.swap(render_point_cloud_buffer_);
      renderdepth_swap_signal = false;
      new_points = true;
    }
  }
  // Update texture
  GLuint tex = color_image_->GetTextureId();
  glEnable(GL_TEXTURE_2D);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, tex);
  {
    std::lock_guard<std::mutex> lock(rendercolor_mutex_);
    color_timestamp = rendercolor_timestamp_;
    if (renderyuv_swap_signal) {
        render_yuv_buffer_.swap(rendershared_yuv_buffer_);
        renderyuv_swap_signal = false;
    }
  }
  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 1920, 1080, GL_LUMINANCE, GL_UNSIGNED_BYTE, render_yuv_buffer_.data());
  glBindTexture(GL_TEXTURE_2D, 0);


  // Querying the depth image's frame transformation based on the depth image's
  // timestamp.
  TangoPoseData pose_start_service_T_device_t0;
  if (util::GetDevicePose(depth_timestamp, &pose_start_service_T_device_t0) !=
      TANGO_SUCCESS) {
    LOGE("SynchronizationApplication: Could not find a valid pose at time %lf"
        " for the depth camera.",
        depth_timestamp);
  }

  // Querying the color image's frame transformation based on the depth image's
  // timestamp.
  TangoPoseData pose_start_service_T_device_t1;
  if (util::GetDevicePose(color_timestamp, &pose_start_service_T_device_t1) !=
      TANGO_SUCCESS) {
    LOGE("SynchronizationApplication: Could not find a valid pose at time %lf"
        " for the color camera.",
        color_timestamp);
  }

  // In the following code, we define t0 as the depth timestamp and t1 as the
  // color camera timestamp.
  glm::mat4 start_service_T_device_t0 =
      util::GetMatrixFromPose(&pose_start_service_T_device_t0);
  glm::mat4 start_service_T_device_t1 =
      util::GetMatrixFromPose(&pose_start_service_T_device_t1);
  glm::mat4 device_t0_T_depth_t0 = device_T_depth_;
  glm::mat4 color_t1_T_device_t1 = glm::inverse(device_T_color_);

  if (pose_start_service_T_device_t1.status_code == TANGO_POSE_VALID) {
    if (pose_start_service_T_device_t0.status_code == TANGO_POSE_VALID) {
      // Note that we are discarding all invalid poses at the moment, another
      // option could be to use the latest pose when the queried pose is
      // invalid.

      // The Color Camera frame at timestamp t0 with respect to Depth
      // Camera frame at timestamp t1.
      glm::mat4 color_image_t1_T_depth_image_t0 =
          color_t1_T_device_t1 * glm::inverse(start_service_T_device_t1) *
          start_service_T_device_t0 * device_t0_T_depth_t0;

      if(gpu_upsample_) {
        depth_image_->RenderDepthToTexture(color_image_t1_T_depth_image_t0,
                                           render_point_cloud_buffer_, new_points);
      } else {
        depth_image_->UpdateAndUpsampleDepth(color_image_t1_T_depth_image_t0,
                                             render_point_cloud_buffer_);
      }
      tracking = true;
    } else {
        tracking = false;
    }
  } else {
      tracking = false;
  }
  main_scene_->Render();
  main_scene_->RenderTrackingStatus(tracking, localized);
}

void SynchronizationApplication::SetGPUUpsample(bool on) {
  gpu_upsample_ = on;
}

}  // namespace rgb_depth_sync

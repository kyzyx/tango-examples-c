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

#include "tango-gl/conversions.h"

#include "rgb-depth-sync/scene.h"

namespace rgb_depth_sync {

// Camera vertical offset value.
static const glm::vec3 kHeightOffset = glm::vec3(0.0f, 1.3f, 0.0f);

// Min/max clamp value of camera observation distance.
static const float kCamViewMinDist = 1.0f;
static const float kCamViewMaxDist = 100.f;

// Scale frustum size for closer near clipping plane.
static const float kFovScaler = 0.1f;

// Color of the ground grid.
static const tango_gl::Color kGridColor(0.85f, 0.85f, 0.85f);

// We'll build our projection matrix for AR off of the intrinsics from the
// color camera.
void Scene::SetCameraIntrinsics(const TangoCameraIntrinsics& cc_intrinsics) {
  float image_width = cc_intrinsics.width;
  float image_height = cc_intrinsics.height;
  // Image plane focal length for x axis.
  float img_fl = static_cast<float>(cc_intrinsics.fx);
  float image_plane_ratio = image_height / image_width;
  float image_plane_dis = 2.0f * img_fl / image_width;
  projection_matrix_ar_ = glm::frustum(
      -1.0f * kFovScaler, 1.0f * kFovScaler, -image_plane_ratio * kFovScaler,
      image_plane_ratio * kFovScaler, image_plane_dis * kFovScaler,
      kCamViewMaxDist);
}

Scene::Scene(ColorImage* color_image, DepthImage* depth_image) {
  OW_T_W_ = tango_gl::conversions::opengl_world_T_tango_world();
  CC_T_OC_ = tango_gl::conversions::color_camera_T_opengl_camera();

  grid_.SetColor(kGridColor);

  camera_texture_drawable_.SetColorTextureId(color_image->GetTextureId());
  camera_texture_drawable_.SetDepthTextureId(depth_image->GetTextureId());
  camera_texture_drawable_.SetBlendAlpha(0.0f);

  trackingcircle_ = new StatusCircle(0.05, 32);
  trackingcircle_->SetColor(0.2,0.3,0.2);
  trackingcircle_->SetPosition(-0.95,0.9);

  localizedcircle_ = new StatusCircle(0.05, 32);
  localizedcircle_->SetColor(0.2,0.2,0.3);
  localizedcircle_->SetPosition(-0.95,0.8);
}

Scene::~Scene() {
    if (trackingcircle_) delete trackingcircle_;
    if (localizedcircle_) delete localizedcircle_;
}

void Scene::SetupViewPort(int w, int h) {
  screen_width_ = w;
  screen_height_ = h;

  if (h == 0 || w == 0) {
    LOGE("The Scene received an invalid height of 0 in SetupViewPort.");
  }
  glViewport(0, 0, screen_width_, screen_height_);
}

// We'll render the scene from a pose.
void Scene::Render() {
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
  camera_texture_drawable_.RenderImage();
}
void Scene::RenderTrackingStatus(int tracking, int localized) {
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    if (tracking > 0) trackingcircle_->SetColor(0,1,0);
    else if (tracking < 0) trackingcircle_->SetColor(0.2,0.5,0.2);
    else trackingcircle_->SetColor(0.5,0.2,0.2);
    if (localized > 0) localizedcircle_->SetColor(0,0,1);
    else if (localized < 0) localizedcircle_->SetColor(0.2,0.2,0.5);
    else localizedcircle_->SetColor(0.2,0.2,0.25);
    glm::mat4 proj(1.0);
    proj[0][0] = screen_height_/(float)screen_width_;
    trackingcircle_->Render(proj);
    localizedcircle_->Render(proj);
}

void Scene::SetDepthAlphaValue(float alpha) {
  camera_texture_drawable_.SetBlendAlpha(alpha);
}

}  // namespace rgb_depth_sync

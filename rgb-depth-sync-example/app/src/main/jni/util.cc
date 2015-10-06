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

#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {

glm::mat4 util::GetMatrixFromPose(const TangoPoseData* pose_data) {
  glm::vec3 translation =
      glm::vec3(pose_data->translation[0], pose_data->translation[1],
                pose_data->translation[2]);
  glm::quat rotation =
      glm::quat(pose_data->orientation[3], pose_data->orientation[0],
                pose_data->orientation[1], pose_data->orientation[2]);
  return glm::translate(glm::mat4(1.0f), translation) *
         glm::mat4_cast(rotation);
}

void util::SetUUIDMetadataValue(const char* uuid, const char* key,
                                         int value_size, const char* value)
{
    char* name;
    TangoAreaDescriptionMetadata metadata;
    if (TangoService_getAreaDescriptionMetadata(uuid, &metadata) !=
            TANGO_SUCCESS) {
        LOGE("TangoService_getAreaDescriptionMetadata(): Failed");
    }
    if (TangoAreaDescriptionMetadata_set(metadata, key, value_size, value) !=
            TANGO_SUCCESS) {
        LOGE("TangoAreaDescriptionMetadata_set(): Failed");
    }
    if (TangoService_saveAreaDescriptionMetadata(uuid, metadata) !=
            TANGO_SUCCESS) {
        LOGE("TangoService_saveAreaDescriptionMetadata(): Failed");
    }
}
char* util::GetUUIDMetadataValue(const char* uuid, const char* key)
{
    size_t size = 0;
    char* name;
    TangoAreaDescriptionMetadata metadata;
    if (TangoService_getAreaDescriptionMetadata(uuid, &metadata) !=
            TANGO_SUCCESS) {
        LOGE("TangoService_getAreaDescriptionMetadata(): Failed");
    }
    if (TangoAreaDescriptionMetadata_get(metadata, key, &size, &name) !=
            TANGO_SUCCESS) {
        LOGE("TangoAreaDescriptionMetadata_get(): Failed");
    }
    return name;
}

}  // namespace rgb_depth_sync

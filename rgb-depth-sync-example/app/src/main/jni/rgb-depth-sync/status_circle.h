#ifndef RGB_DEPTH_SYNC_STATUS_CIRCLE_H
#define RGB_DEPTH_SYNC_STATUS_CIRCLE_H

#include <tango-gl/util.h>
#include "rgb-depth-sync/shader.h"

namespace rgb_depth_sync {
class StatusCircle {
 public:
  StatusCircle(float radius, int resolution);
  ~StatusCircle();
  // Render the color texture on screen.
  void Render(glm::mat4 proj);

  void SetColor(float red, float green, float blue) {
      r = red;
      g = green;
      b = blue;
  }

  void SetPosition(float x, float y) {
      model[3][0] = x;
      model[3][1] = y;
  }

 private:
  float r, g, b;
  glm::mat4 model;

  GLuint uniform_statuscolor;
  GLuint uniform_mvp;
  GLuint attrib_vertices_;

  GLuint shader_program_;
  GLuint render_buffers_;

  std::vector<GLfloat> vertices;
  std::vector<GLushort> indices;
};
}  // namespace rgb_depth_sync

#endif  // RGB_DEPTH_SYNC_STATUS_CIRCLE_H

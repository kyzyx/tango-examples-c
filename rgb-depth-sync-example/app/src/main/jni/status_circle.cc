#include <rgb-depth-sync/status_circle.h>

namespace rgb_depth_sync {

StatusCircle::StatusCircle(float radius, int resolution) {
  shader_program_ =
      tango_gl::util::CreateProgram(rgb_depth_sync::shader::kStatusCircleVert,
                                    rgb_depth_sync::shader::kStatusCircleFrag);
  if (!shader_program_) {
    LOGE("Could not create shader program for StatusCircle.");
  }
  vertices.reserve(3 * (resolution+1));
  vertices.push_back(0);
  vertices.push_back(0);
  vertices.push_back(0);
  float delta_theta = M_PI * 2.0f / static_cast<float>(resolution);
  for (int i = 0; i <= resolution; i++) {
    float theta = delta_theta * static_cast<float>(i);
    vertices.push_back(cos(theta) * radius);
    vertices.push_back(sin(theta) * radius);
    vertices.push_back(0);
  }

  glGenBuffers(1, &render_buffers_);
  // Allocate vertices buffer.
  glBindBuffer(GL_ARRAY_BUFFER, render_buffers_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * vertices.size(),
          vertices.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  // Assign the vertices attribute data.
  attrib_vertices_ = glGetAttribLocation(shader_program_, "vertex");
  glBindBuffer(GL_ARRAY_BUFFER, render_buffers_);
  glEnableVertexAttribArray(attrib_vertices_);
  glVertexAttribPointer(attrib_vertices_, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  uniform_statuscolor = glGetUniformLocation(shader_program_, "color");
  uniform_mvp = glGetUniformLocation(shader_program_, "mvp");

  model = glm::mat4(1.0);
}

StatusCircle::~StatusCircle() {
  glDeleteShader(shader_program_);
}

void StatusCircle::Render(glm::mat4 proj) {
  glDisable(GL_DEPTH_TEST);

  glUseProgram(shader_program_);

  glm::mat4 mvp = model*proj; // Wrong order, okay for translation only
  glUniform4f(uniform_statuscolor, r, g, b, 1);
  glUniformMatrix4fv(uniform_mvp, 1, GL_FALSE, glm::value_ptr(mvp));

  // Bind vertices buffer.
  glBindBuffer(GL_ARRAY_BUFFER, render_buffers_);
  glEnableVertexAttribArray(attrib_vertices_);
  glVertexAttribPointer(attrib_vertices_, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size()/3);
  tango_gl::util::CheckGlError("StatusCircle glDrawElements");
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  glUseProgram(0);
  tango_gl::util::CheckGlError("StatusCircle::render");
}

}  // namespace rgb_depth_sync

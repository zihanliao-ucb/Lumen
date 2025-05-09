#ifndef CGL_GLSCENE_SPHERE_H
#define CGL_GLSCENE_SPHERE_H

#include "scene.h"
#include "corecrt_math_defines.h"

#include "../collada/sphere_info.h"

namespace CGL { namespace GLScene {

class Sphere : public SceneObject {
 public:
  Sphere(const Collada::SphereInfo& sphereInfo, const Vector3D position,
         const double scale);

  void set_draw_styles(DrawStyle *defaultStyle, DrawStyle *hoveredStyle,
                       DrawStyle *selectedStyle);

  void render_in_opengl() const;

  void render_debugger_node();
	std::vector<std::pair<Vector3D, Vector3D>> sample_points(double pixelArea);

  BBox get_bbox();

  // All functions that are unused, because spheres can't be selected yet.
  double test_selection(const Vector2D& p, const Matrix4x4& worldTo3DH,
                        double minW) {
    return -1;
  }
  void confirm_hover() { }
  void confirm_select() { }
  void invalidate_hover() { }
  void invalidate_selection() { }
  void get_selection_info(SelectionInfo *selectionInfo) { }
  void drag_selection(float dx, float dy, const Matrix4x4& worldTo3DH) { }
  MeshView *get_mesh_view() { return nullptr; }

  BSDF* get_bsdf();
  SceneObjects::SceneObject *get_static_object();

private:
  double r;
  Vector3D p;
  BSDF* bsdf;
  DrawStyle *style;

};

} // namespace GLScene
} // namespace CGL

#endif //CGL_GLSCENE_SPHERE_H

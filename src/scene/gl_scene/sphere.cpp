#include "sphere.h"

#include "scene/object.h"
#include "util/sphere_drawing.h"

#include "application/visual_debugger.h"

#include "pathtracer/bsdf.h"

namespace CGL { namespace GLScene {

Sphere::Sphere(const Collada::SphereInfo& info, 
               const Vector3D position, const double scale) : 
  p(position), r(info.radius * scale) { 
  if (info.material) {
    bsdf = info.material->bsdf;
  } else {
    bsdf = new DiffuseBSDF(Vector3D(0.5f,0.5f,0.5f));    
  }
}

void Sphere::set_draw_styles(DrawStyle *defaultStyle, DrawStyle *hoveredStyle,
                             DrawStyle *selectedStyle) {
  style = defaultStyle;
}

void Sphere::render_in_opengl() const {
  Misc::draw_sphere_opengl(p, r);
}

std::vector<std::pair<Vector3D, Vector3D>> Sphere::sample_points(double pixelArea) {
  std::vector<std::pair<Vector3D, Vector3D>> points;
	// Sample points uniformly on the sphere surface
	for (int i = 0; i < 100; i++) {
		double theta = rand() / (double)RAND_MAX * M_PI;
		double phi = rand() / (double)RAND_MAX * 2 * M_PI;
		double x = r * sin(theta) * cos(phi);
		double y = r * sin(theta) * sin(phi);
		double z = r * cos(theta);
    points.push_back({ Vector3D(p.x + x, p.y + y, p.z + z), Vector3D(x, y, z)});
	}
	return points;
}

void Sphere::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Sphere"))
  {
    DragDouble("Radius", &r, 0.005);
    DragDouble3("Position", &p[0], 0.005);

    if (bsdf) bsdf->render_debugger_node();

    ImGui::TreePop();
  }
}

BBox Sphere::get_bbox() {
  return BBox(p.x - r, p.y - r, p.z - r, p.x + r, p.y + r, p.z + r);
}

BSDF* Sphere::get_bsdf() {
  return bsdf;
}

SceneObjects::SceneObject *Sphere::get_static_object() {
  return new SceneObjects::SphereObject(p, r, bsdf);
}

} // namespace GLScene
} // namespace CGL

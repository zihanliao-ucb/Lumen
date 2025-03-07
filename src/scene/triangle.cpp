#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray& r) const {
  Vector3D e1 = p2 - p1;
  Vector3D e2 = p3 - p1;
  Vector3D s1 = cross(r.d, e2);
  double det = dot(e1, s1);

  if (det > -1e-6 && det < 1e-6) {
    return false; // Ray is parallel to the triangle
  }

  Vector3D s = r.o - p1;
  double b1 = dot(s, s1) / det;
  if (b1 < 0.0 || b1 > 1.0) return false;

  Vector3D s2 = cross(s, e1);
  double b2 = dot(r.d, s2) / det;
  if (b2 < 0.0 || (b1 + b2) > 1.0) return false;

  double t = dot(e2, s2) / det;
  if (t < r.min_t || t > r.max_t) return false;

  return true;
}


bool Triangle::intersect(const Ray& r, Intersection* isect) const {
  Vector3D e1 = p2 - p1;
  Vector3D e2 = p3 - p1;
  Vector3D s1 = cross(r.d, e2);
  double det = dot(e1, s1);

  if (det > -1e-6 && det < 1e-6) {
    return false; // Ray is parallel to the triangle
  }

  double inv_det = 1.0 / det;
  Vector3D s = r.o - p1;
  double b1 = dot(s, s1) * inv_det;
  if (b1 < 0.0 || b1 > 1.0) return false;

  Vector3D s2 = cross(s, e1);
  double b2 = dot(r.d, s2) * inv_det;
  if (b2 < 0.0 || (b1 + b2) > 1.0) return false;

  double t = dot(e2, s2) * inv_det;
  if (t < r.min_t || t > r.max_t) return false;

  r.max_t = min(t, r.max_t);

  if (isect) {
    isect->t = t;
    isect->primitive = this;
    isect->n = (1 - b1 - b2) * n1 + b1 * n2 + b2 * n3; // Interpolated normal
    isect->bsdf = get_bsdf();
  }

  return true;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL

#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  BBox bbox;
	size_t num_primitives = 0;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
		num_primitives++;
  }

  BVHNode *node = new BVHNode(bbox);

	if (num_primitives <= max_leaf_size) {
		node->start = start;
		node->end = end;
		return node;
	}

  Vector3D extent = bbox.extent;
  int split_axis = (extent.x > extent.y && extent.x > extent.z) ? 0 : (extent.y > extent.z) ? 1 : 2;
  std::sort(start, end, [split_axis](Primitive* a, Primitive* b) {
    return a->get_bbox().centroid()[split_axis] < b->get_bbox().centroid()[split_axis];
  });

  double best_cost = std::numeric_limits<double>::infinity();
  auto best_split = start + num_primitives / 2;

  std::vector<BBox> left_bbox(num_primitives), right_bbox(num_primitives);
  BBox left_accum, right_accum;
  for (size_t i = 0; i < num_primitives; ++i) {
    left_accum.expand((*(start + i))->get_bbox());
    left_bbox[i] = left_accum;
  }

  for (size_t i = num_primitives; i > 0; --i) {
    right_accum.expand((*(start + i - 1))->get_bbox());
    right_bbox[i - 1] = right_accum;
  }

  // Evaluate SAH cost at each possible split point
  for (size_t i = 1; i < num_primitives; ++i) {
    double left_area = left_bbox[i - 1].surface_area();
    double right_area = right_bbox[i].surface_area();
    double cost = left_area * i + right_area * (num_primitives - i);

    if (cost < best_cost) {
      best_cost = cost;
      best_split = start + i;
    }
  }

  if (best_split == start || best_split == end) {
    best_split = start + num_primitives / 2;
  }

  node->l = construct_bvh(start, best_split, max_leaf_size);
  node->r = construct_bvh(best_split, end, max_leaf_size);

  return node;
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
  if (!node) return false;

  double t0 = ray.min_t, t1 = ray.max_t;
  if (!node->bb.intersect(ray, t0, t1)) return false;

	if (node->isLeaf()) {
		for (auto p = node->start; p != node->end; p++) {
			total_isects++;
			if ((*p)->has_intersection(ray))
				return true;
		}
		return false;
	}

	bool hit = has_intersection(ray, node->l);
	if (hit) return true;
	return has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  if (!node) return false;

  double t0 = ray.min_t, t1 = ray.max_t;

  if (!node->bb.intersect(ray, t0, t1)) return false;

	if (node->isLeaf()) {
    bool hit = false;
		for (auto p = node->start; p != node->end; p++) {
			total_isects++;
      if ((*p)->intersect(ray, i))
				hit = true;
		}
		return hit;
	}

  bool hit_left = false, hit_right = false;
  double t0_l = t0, t1_l = t1, t0_r = t0, t1_r = t1;
  bool left_intersects = node->l->bb.intersect(ray, t0_l, t1_l);
  bool right_intersects = node->r->bb.intersect(ray, t0_r, t1_r);

  if (left_intersects && right_intersects) {
	  if (t0_l < t0_r) {
		  hit_left = intersect(ray, i, node->l);
		  if (!hit_left)
			  hit_right = intersect(ray, i, node->r);
      else if (t1_l > t0_r)
			  hit_right = intersect(ray, i, node->r);
	  }
	  else {
		  hit_right = intersect(ray, i, node->r);
		  if (!hit_right)
			  hit_left = intersect(ray, i, node->l);
		  else if (t1_r > t0_l)
			  hit_left = intersect(ray, i, node->l);
	  }
  }
  else if (left_intersects) {
	  hit_left = intersect(ray, i, node->l);
  }
  else if (right_intersects) {
	  hit_right = intersect(ray, i, node->r);
  }

  return hit_left || hit_right;
}

} // namespace SceneObjects
} // namespace CGL

#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out(0.0);

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  for (int i = 0; i < num_samples; i++) {
    // Sample a direction uniformly from the hemisphere
    Vector3D w_in = hemisphereSampler->get_sample();
    double pdf = 1.0 / (2.0 * PI); // Uniform sampling over hemisphere

    // Convert sampled direction to world space
    Vector3D w_in_world = o2w * w_in;

    // Cast a shadow ray
    Ray shadow_ray(hit_p, w_in_world);
		shadow_ray.min_t = EPS_F;
    Intersection light_isect;
    if (!bvh->intersect(shadow_ray, &light_isect)) continue; // Skip if no intersection

    // Compute contribution if the intersected object is an emissive light
    Vector3D L_i = light_isect.bsdf->get_emission();
    L_out += isect.bsdf->f(w_out, w_in) * L_i * abs_cos_theta(w_in) / pdf;
  }

  return L_out / num_samples; // Average over all samples

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out(0.0);

  for (const auto& light : scene->lights) {
    int num_samples = (light->is_delta_light()) ? 1 : ns_area_light; // More samples for area lights

    for (int i = 0; i < num_samples; i++) {
      Vector3D wiw;
      double dist_to_light, pdf;
      Vector3D L_i = light->sample_L(hit_p, &wiw, &dist_to_light, &pdf);

      if (pdf < EPS_F) continue; // Skip invalid samples

      Vector3D w_in = w2o * wiw; // Convert light direction to local space
      if (w_in.z < 0) continue; // Ignore directions below the surface

      // Check if the point is in shadow
      Ray shadow_ray(hit_p, wiw);
			shadow_ray.min_t = EPS_F;
      shadow_ray.max_t = std::max(dist_to_light - EPS_F, shadow_ray.min_t);
      if (bvh->has_intersection(shadow_ray)) continue; // Shadowed

      // Compute the contribution of this sample
      L_out += isect.bsdf->f(w_out, w_in) * L_i * abs_cos_theta(w_in) / pdf;
    }

    if (!light->is_delta_light()) L_out /= num_samples; // Average non-delta lights
  }

  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
	if (!isect.bsdf) return Vector3D(0, 0, 0);
	return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`
  if (direct_hemisphere_sample) {
    return estimate_direct_lighting_hemisphere(r, isect);
  }
  else {
    return estimate_direct_lighting_importance(r, isect);
  }
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
	if (max_ray_depth == 0) return Vector3D(0.0);

  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.
  Vector3D direct_light = one_bounce_radiance(r, isect);

  if (r.depth >= max_ray_depth - 1) {
    return direct_light; // If only last bounce is needed, return at max depth
  }

  if (isAccumBounces) {
    L_out += direct_light; // Accumulate direct lighting
  }

  Vector3D w_in;
  double pdf;
  Vector3D f = isect.bsdf->sample_f(w_out, &w_in, &pdf);

  if (pdf < EPS_F || f.norm() < EPS_F) return L_out; // Avoid division by zero

  Vector3D w_in_world = o2w * w_in;
  Ray new_ray(hit_p, w_in_world);
	new_ray.min_t = EPS_F;
  new_ray.depth = r.depth + 1;

  double rr_prob = 0.35;
  if (new_ray.depth >= max_ray_depth || coin_flip(rr_prob)) return L_out;

  Intersection new_isect;
  if (bvh->intersect(new_ray, &new_isect)) {
    Vector3D indirect = at_least_one_bounce_radiance(new_ray, new_isect);
    L_out += (f * indirect * abs_cos_theta(w_in)) / (pdf * (1 - rr_prob)); // Accumulate all bounces
  }

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;


  //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  //return L_out;
  if (!isAccumBounces && max_ray_depth > 0)
    return at_least_one_bounce_radiance(r, isect);
  else
    return zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  int num_samples = 0;        // Current number of samples taken
  Vector2D origin = Vector2D(x, y); // Bottom-left corner of the pixel

  Vector3D sampleColor(0, 0, 0);  // Accumulated radiance
  double s1 = 0, s2 = 0;          // Accumulators for mean and variance

  for (int i = 0; i < ns_aa; i++) {
    Vector2D sample = gridSampler->get_sample();
    Vector2D p = origin + (sample + Vector2D(0.5, 0.5));
    Ray r = camera->generate_ray(p.x / sampleBuffer.w, p.y / sampleBuffer.h);
    Vector3D radiance = est_radiance_global_illumination(r);

    double illum = radiance.illum(); // Convert to scalar luminance
    s1 += illum;
    s2 += illum * illum;

    sampleColor += radiance;
    num_samples++;

    // Perform adaptive sampling check every samplesPerBatch
    if (num_samples % samplesPerBatch == 0) {
      double mean = s1 / num_samples;
      double variance = (s2 - (s1 * s1) / num_samples) / (num_samples - 1);
      double stddev = sqrt(variance);
			double I = 1.96 * stddev / sqrt(num_samples);

      // Check convergence condition
      if (I <= maxTolerance * mean) {
        break; // Stop sampling if converged
      }
    }
  }

  sampleBuffer.update_pixel(sampleColor / num_samples, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
}


void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL

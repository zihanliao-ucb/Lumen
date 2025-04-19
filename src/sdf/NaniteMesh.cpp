#include "NaniteMesh.h"

namespace CGL {

	NaniteMesh::NaniteMesh(GLScene::Mesh* mesh) : mesh(mesh) {
		// Initialize the sdf
		initSDF();
		// Compute the sdf
		for (int i = 0; i < 10; ++i) {
			computeSDF();
		}
		std::vector<Eigen::Vector3f> vertices;
		for (VertexIter v = mesh->mesh.verticesBegin(); v != mesh->mesh.verticesEnd(); ++v) {
			vertices.emplace_back(v->position.x, v->position.y, v->position.z);
		}
		// Get the cards
		splitOBBs(cards, vertices, 12);
		computeCards(0.001f);
	}

	NaniteMesh::~NaniteMesh() {
		// Clean the sdf data
		sdf.clear();
		normals.clear();
		seedPoints.clear();
	}

	void NaniteMesh::setShader(GLuint sdfarr, GLuint normalarr, GLuint seedarr, GLuint shader) {
		sdfArray = sdfarr;
		normalArray = normalarr;
		seedArray = seedarr;
		shaderProgram = shader;
	}

	void NaniteMesh::initSDF() {
		// Get the bounding box of the mesh
		BBox bbox = mesh->get_bbox();
		Vector3D min = bbox.min;
		Vector3D max = bbox.max;
		Vector3D size = max - min;
		// Get the number of points in the mesh
		int numPoints = mesh->mesh.nVertices();
		std::cout << "Number of points: " << numPoints << std::endl;
		// Get the size of the sdf
		float sdfVoxelLength = std::sqrt(static_cast<float>((size.x * size.y + size.y * size.z + size.z * size.x) / numPoints));
		float fx = static_cast<float>(size.x);
		float fy = static_cast<float>(size.y);
		float fz = static_cast<float>(size.z);
		vx = fx < 0.2f * sdfVoxelLength ? std::max(fx, 0.01f) : sdfVoxelLength;
		sx = std::min(std::max(static_cast<int>(std::ceil(size.x / vx)), 1), 128);
		vx = std::max(static_cast<float>(size.x) / sx, 0.01f);
		vy = fy < 0.2f * sdfVoxelLength ? std::max(fy, 0.01f) : sdfVoxelLength;
		sy = std::min(std::max(static_cast<int>(std::ceil(size.y / vy)), 1), 128);
		vy = std::max(static_cast<float>(size.y) / sy, 0.01f);
		vz = fz < 0.2f * sdfVoxelLength ? std::max(fz, 0.01f) : sdfVoxelLength;
		sz = std::min(std::max(static_cast<int>(std::ceil(size.z / vz)), 1), 128);
		vz = std::max(static_cast<float>(size.z) / sz, 0.01f);
		std::cout << "Voxel Length: " << vx << " " << vy << " " << vz << sdfVoxelLength << " SDF size: " << sx << " " << sy << " " << sz << std::endl;
		// Set the origin of the sdf
		ox = static_cast<float>(min.x);
		oy = static_cast<float>(min.y);
		oz = static_cast<float>(min.z);
		// Set the transformation matrix
		mat[0] = 1.0f / vx; mat[1] = 0.0f; mat[2] = 0.0f;
		mat[3] = 0.0f; mat[4] = 1.0f / vy; mat[5] = 0.0f;
		mat[6] = 0.0f; mat[7] = 0.0f; mat[8] = 1.0f / vz;
		// Initialize the sdf and normals
		sdf.resize(sx * sy * sz, std::numeric_limits<float>::max());
		normals.resize(sx * sy * sz, Vector3D(0.0f, 0.0f, 0.0f));
		seedPoints.resize(sx * sy * sz, Vector3D(-1.0f, -1.0f, -1.0f));
		// Compute the seed points
		for (VertexIter v = mesh->mesh.verticesBegin(); v != mesh->mesh.verticesEnd(); ++v) {
			Vector3D p = v->position;
			int x = std::min(std::max(static_cast<int>(std::floor((p.x - ox) / vx)), 0), sx - 1);
			int y = std::min(std::max(static_cast<int>(std::floor((p.y - oy) / vy)), 0), sy - 1);
			int z = std::min(std::max(static_cast<int>(std::floor((p.z - oz) / vz)), 0), sz - 1);
			Vector3D p_grid = Vector3D((x + 0.5) * vx,
				(y + 0.5) * vy,
				(z + 0.5) * vz);
			sdf[x + y * sx + z * sx * sy] = 0.0f;
			normals[x + y * sx + z * sx * sy] = v->normal;
			seedPoints[x + y * sx + z * sx * sy] = p_grid;
		}
		float maxUnit = std::min({ vx * vy, vy * vz, vz * vx });
		std::vector<std::pair<Vector3D, Vector3D>> points = mesh->sample_points(maxUnit);
		for (auto& point : points) {
			Vector3D p = point.first;
			Vector3D normal = point.second;
			int x = std::min(std::max(static_cast<int>(std::floor((p.x - ox) / vx)), 0), sx - 1);
			int y = std::min(std::max(static_cast<int>(std::floor((p.y - oy) / vy)), 0), sy - 1);
			int z = std::min(std::max(static_cast<int>(std::floor((p.z - oz) / vz)), 0), sz - 1);
			Vector3D p_grid = Vector3D((x + 0.5) * vx,
				(y + 0.5) * vy,
				(z + 0.5) * vz);
			sdf[x + y * sx + z * sx * sy] = 0.0f;
			normals[x + y * sx + z * sx * sy] = normal;
			seedPoints[x + y * sx + z * sx * sy] = p_grid;
		}
	}

  void NaniteMesh::computeSDF() {
    int maxDim = std::max({ sx, sy, sz });
    int jump = 1 << static_cast<int>(std::floor(std::log2(maxDim)));

    std::vector<float> newSDF = sdf;
    std::vector<Vector3D> newNormals = normals;
    std::vector<Vector3D> newSeeds = seedPoints;

    while (jump >= 1) {
      for (int z = 0; z < sz; ++z) {
        for (int y = 0; y < sy; ++y) {
          for (int x = 0; x < sx; ++x) {
            int idx = x + y * sx + z * sx * sy;

            Vector3D p_grid = Vector3D(
              (x + 0.5f) * vx,
              (y + 0.5f) * vy,
              (z + 0.5f) * vz
            );

            float minDist = std::abs(sdf[idx]);
            Vector3D minNormal = normals[idx];
            Vector3D minSeed = seedPoints[idx];

            for (int dz = -1; dz <= 1; ++dz) {
              for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                  if (dx == 0 && dy == 0 && dz == 0) continue;
                  int nx = x + dx * jump;
                  int ny = y + dy * jump;
                  int nz = z + dz * jump;

                  if (nx < 0 || nx >= sx || ny < 0 || ny >= sy || nz < 0 || nz >= sz)
                    continue;

                  int nidx = nx + ny * sx + nz * sx * sy;
                  Vector3D neighborSeed = seedPoints[nidx];
                  Vector3D neighborNormal = normals[nidx];

                  if (neighborSeed.x < 0.0f) continue; // not a seed

                  float dist = (neighborSeed - p_grid).norm();
                  if (dist < minDist) {
                    minDist = dist;
                    minSeed = neighborSeed;
                    minNormal = neighborNormal;
                  }
                }
              }
            }

            // Compute sign based on normal direction
            float sign = (dot(minNormal, p_grid - minSeed) < 0.0f) ? -1.0f : 1.0f;

            newSDF[idx] = sign * minDist;
            newNormals[idx] = minNormal;
            newSeeds[idx] = minSeed;
          }
        }
      }

      sdf = newSDF;
      normals = newNormals;
      seedPoints = newSeeds;

      jump /= 2;
    }
  }

	OBB NaniteMesh::computeOBB(const std::vector<Eigen::Vector3f>& points) {
		Eigen::Vector3f mean = Eigen::Vector3f::Zero();
		for (const auto& p : points) mean += p;
		mean /= points.size();

		Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
		for (const auto& p : points) {
			Eigen::Vector3f d = p - mean;
			cov += d * d.transpose();
		}
		cov /= points.size();

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
		Eigen::Matrix3f basis = solver.eigenvectors(); // columns = eigenvectors

		// project to PCA space
		Eigen::Vector3f minP = Eigen::Vector3f::Constant(FLT_MAX);
		Eigen::Vector3f maxP = Eigen::Vector3f::Constant(-FLT_MAX);
		for (const auto& p : points) {
			Eigen::Vector3f local = basis.transpose() * (p - mean);
			minP = minP.cwiseMin(local);
			maxP = maxP.cwiseMax(local);
		}

		Eigen::Vector3f extent = maxP - minP;
		Eigen::Vector3f origin = mean + basis * minP;

		return OBB{ origin, basis, extent, points };
	}

	void NaniteMesh::splitOBBs(std::vector<OBB>& result, const std::vector<Eigen::Vector3f>& points, int maxBoxes) {
		std::priority_queue<OBB, std::vector<OBB>> queue;
		queue.push(computeOBB(points));

		while (!queue.empty() && result.size() + queue.size() < maxBoxes) {
			OBB current = queue.top(); queue.pop();
			if (current.points.size() <= 4) {
				current.extent = current.extent.cwiseMax(Eigen::Vector3f(0.01f, 0.01f, 0.01f));
				result.push_back(current);
				continue;
			}

			Eigen::Vector3f axis = current.rotation.col(2); // splitting along PCA axis

			std::vector<std::pair<float, Eigen::Vector3f>> projections;
			for (const auto& p : current.points) {
				float t = (p - current.origin).dot(axis);
				projections.emplace_back(t, p);
			}

			std::sort(projections.begin(), projections.end(),
				[](const std::pair<float, Eigen::Vector3f>& a, const std::pair<float, Eigen::Vector3f>& b) {
					return a.first < b.first;
				});

			int mid = projections.size() / 2;
			std::vector<Eigen::Vector3f> left, right;
			for (size_t i = 0; i < projections.size(); ++i) {
				if (i < mid) left.push_back(projections[i].second);
				else right.push_back(projections[i].second);
			}

			if (!left.empty()) queue.push(computeOBB(left));
			if (!right.empty()) queue.push(computeOBB(right));
		}

		while (!queue.empty()) {
			OBB final = queue.top(); queue.pop();
			final.extent = final.extent.cwiseMax(Eigen::Vector3f(0.01f, 0.01f, 0.01f));
			result.push_back(final);
		}
	}

	void NaniteMesh::computeCards(float area) {
		// Compute the card size and the transformation matrix
		float nominalSize = std::sqrt(area);
		int bias = 0;
		for (int i = 0; i < cards.size(); ++i) {
			const OBB& obb = cards[i];
			NaniteCard card;
			card.box = obb;
			card.sx = std::min(std::max(static_cast<int>(std::ceil(obb.extent.x() / nominalSize)), 1), 128);
			card.sy = std::min(std::max(static_cast<int>(std::ceil(obb.extent.y() / nominalSize)), 1), 128);
			card.sz = std::min(std::max(static_cast<int>(std::ceil(obb.extent.z() / nominalSize)), 1), 128);
			card.vx = std::max(obb.extent.x() / card.sx, 0.01f);
			card.vy = std::max(obb.extent.y() / card.sy, 0.01f);
			card.vz = std::max(obb.extent.z() / card.sz, 0.01f);
			Eigen::Matrix3f scale;
			scale << 1.0f / card.vx, 0, 0,
				0, 1.0f / card.vy, 0,
				0, 0, 1.0f / card.vz;
			Eigen::Matrix3f transform = scale * card.box.rotation.transpose();
			for (int col = 0; col < 3; ++col) {
				for (int row = 0; row < 3; ++row) {
					card.mat[col * 3 + row] = transform(row, col);
					//card.mat[row * 3 + col] = transform(row, col);
				}
			}

			Eigen::Vector3f test = card.box.origin + card.box.rotation * (card.box.extent * 0.1f);
			Eigen::Vector3f local = transform * (test - card.box.origin);
			std::cout << "Mapped to voxel space: " << local.transpose() << std::endl;
			std::cout << "Card size: " << card.sx << " " << card.sy << " " << card.sz << std::endl;

			card.normals.resize(card.sx * card.sy * card.sz * 3, 0.0f);
			card.bias = bias;
			nanite_cards.push_back(card);
			bias += card.sx * card.sy * card.sz;
		}
		nCards = nanite_cards.size();
		nCardsVoxels = bias;

		// Compute the normals for cards
		std::vector<std::pair<Vector3D, Vector3D>> points = mesh->sample_points(area);
		for (std::pair<Vector3D, Vector3D>& point : points) {
			Eigen::Vector3f p(
				static_cast<float>(point.first.x),
				static_cast<float>(point.first.y),
				static_cast<float>(point.first.z)
			);
			Eigen::Vector3f normal(
				static_cast<float>(point.second.x),
				static_cast<float>(point.second.y),
				static_cast<float>(point.second.z)
			);

			for (int i = 0; i < nanite_cards.size(); ++i) {
				NaniteCard& card = nanite_cards[i];
				Eigen::Vector3f offset = p - card.box.origin;

				// Reconstruct column-major matrix from card.mat[]
				Eigen::Matrix3f M;
				M << card.mat[0], card.mat[3], card.mat[6],
					card.mat[1], card.mat[4], card.mat[7],
					card.mat[2], card.mat[5], card.mat[8];

				Eigen::Vector3f local = M * offset;
				Eigen::Vector3i idx = local.array().floor().cast<int>();

				if ((idx.array() >= 0).all() &&
					(idx.array() < Eigen::Vector3i(card.sx, card.sy, card.sz).array()).all()) {
					int index = idx.x() + idx.y() * card.sx + idx.z() * card.sx * card.sy;
					card.normals[index * 3 + 0] = normal.x();
					card.normals[index * 3 + 1] = normal.y();
					card.normals[index * 3 + 2] = normal.z();
				}
			}
		}
	}
};
#include "CGL/CGL.h"
#include "CGL/color.h"
#include "GL/glew.h"

#include "pathtracer/camera.h"
#include "scene/gl_scene/scene.h"
#include "scene/gl_scene/mesh.h" 
#include "Eigen/Dense"

#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <random>
#include <limits>
#include <unordered_set>

#define MAX_BOX_LENGTH (128)
#define MIN_VOXEL_LENGTH (0.005f)


namespace CGL {
	// Helper fucntions
	inline Eigen::Vector3f Vector3f_fromCGL(const Vector3D& v) {
		return Eigen::Vector3f(v.x, v.y, v.z);
	}

	struct OBB {
		Eigen::Vector3f origin; // min corner of the box
		Eigen::Matrix3f rotation; // local basis: columns are axes
		Eigen::Vector3f extent;   // size along each axis
		std::vector<Eigen::Vector3f> points; // points in this cluster

		float getVolume() const {
			return extent.prod();
		};

		float getSurfaceArea() const {
			return 2 * (extent.x() * extent.y() + extent.y() * extent.z() + extent.x() * extent.z());
		};

		bool operator<(const OBB& other) const {
			return (getVolume() / points.size()) < (other.getVolume() / other.points.size()); // Larger boxes go to the top
		};
	};
	
	struct NaniteCard {
		OBB box; // bounding box of the card which contains points data
		Eigen::Vector3i size; // size of the card
		int bias; // bias of the card
		Eigen::Vector3f voxelSize; // size of each voxel in the card
		std::vector<Eigen::Vector3f> positions;
		std::vector<Eigen::Vector3f> normals;
		std::vector<Eigen::Vector3f> tangents;

		void setSize(float nominalSize) {
			float VoxelLength = std::sqrt(box.getSurfaceArea() / box.points.size());
			VoxelLength = std::min(VoxelLength, nominalSize);
			auto computeDim = [=](float extent) -> int {
				return std::clamp(static_cast<int>(std::ceil(extent / VoxelLength)), 1, MAX_BOX_LENGTH);
			};
			size = Eigen::Vector3i(
				computeDim(box.extent.x()),
				computeDim(box.extent.y()),
				computeDim(box.extent.z())
			);
			voxelSize = box.extent.cwiseQuotient(size.cast<float>()).cwiseMax(MIN_VOXEL_LENGTH);
		}

		Eigen::Matrix3f getTransform() const {
			Eigen::Matrix3f scale = voxelSize.cwiseInverse().asDiagonal();
			Eigen::Matrix3f M = scale * box.rotation.transpose();
			return M;
		}

		int getIndex(const Eigen::Vector3f& local) const {
			Eigen::Vector3i idx = local.array().floor().cast<int>();
			if ((idx.array() >= 0).all() &&
				(idx.array() < size.array()).all()) {
				return idx.x() + idx.y() * size.x() + idx.z() * size.x() * size.y();
			}
			return -1;
		}

		bool insert(const Eigen::Vector3f& local, const Eigen::Vector3f& pnormal) {
			int idx = getIndex(local);
			if (idx >= 0) {
				normals[idx] = pnormal;
				return true;
			}
			return false;
		}

		void inflate(const Eigen::Vector3f& p, const Eigen::Vector3f& pnormal) {
			Eigen::Vector3f offset = p - box.origin;
			Eigen::Matrix3f M = getTransform();
			Eigen::Vector3f local = M * offset;
			insert(local, pnormal);
			Eigen::Vector3f pbias = M * pnormal;
			insert(local + voxelSize.x() / 2 * pbias, pnormal);
			//insert(local + voxelSize.y() * pbias, pnormal);
			insert(local - voxelSize.x() / 2 * pbias, pnormal);
			//insert(local - voxelSize.y() * pbias, pnormal);
		}

		void generatePositions() {
			for (int i = 0; i < size.x(); ++i) {
				for (int j = 0; j < size.y(); ++j) {
					for (int k = 0; k < size.z(); ++k) {
						int idx = i + j * size.x() + k * size.x() * size.y();
						positions[idx] = box.origin + box.rotation * voxelSize.cwiseProduct(Eigen::Vector3f(i + 0.5, j + 0.5, k + 0.5));
					}
				}
			}
		}

		void generateTangents() {
			for (int i = 0; i < normals.size(); ++i) {
				Eigen::Vector3f normal = normals[i];
				if (normal.norm() > 0.0f) {
					Eigen::Vector3f tangent = normal.cross(Eigen::Vector3f(1.0f, 0.0f, 0.0f));
					if (tangent.norm() < 1e-6) {
						tangent = normal.cross(Eigen::Vector3f(0.0f, 1.0f, 0.0f));
					}
					tangent.normalize();
					tangents[i] = tangent;
				}
				else {
					tangents[i] = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
				}
			}
		}

		std::vector<float> getPointData() {
			generatePositions();
			generateTangents();
			std::vector<float> data(normals.size() * 9);
			for (size_t i = 0; i < normals.size(); ++i) {
				data[i * 9 + 0] = positions[i].x();
				data[i * 9 + 1] = positions[i].y();
				data[i * 9 + 2] = positions[i].z();
				data[i * 9 + 3] = normals[i].x();
				data[i * 9 + 4] = normals[i].y();
				data[i * 9 + 5] = normals[i].z();
				data[i * 9 + 6] = tangents[i].x();
				data[i * 9 + 7] = tangents[i].y();
				data[i * 9 + 8] = tangents[i].z();
			}
			return data;
		}

		void resetData() {
			positions.resize(size.x() * size.y() * size.z(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
			normals.resize(size.x() * size.y() * size.z(), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
			tangents.resize(size.x() * size.y() * size.z(), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
		}
	};


	class NaniteMesh
	{
	public:
		NaniteMesh(GLScene::Mesh* mesh);
		~NaniteMesh();

		void setShader(GLuint sdfarr, GLuint normalarr, GLuint seedarr, GLuint shader);
		OBB computeOBB(const std::vector<Eigen::Vector3f>& points);
		void splitOBBs(std::vector<OBB>& result, const std::vector<Eigen::Vector3f>& points, int maxBoxes);

		int sx, sy, sz; // size of the sdf
		int sdfBias; // bias of the sdf
		int nCards, cardBias; // number of cards and bias of the cards
		int nCardsVoxels; // number of voxels in the cards

		float vx, vy, vz; // size of each voxel in the sdf
		float ox, oy, oz; // offset of the sdf
		float mat[9]; // transformation matrix

		std::vector<float> sdf; // sdf data
		std::vector<Vector3D> normals; // normal data
		std::vector<Vector3D> seedPoints; // seed points for the sdf

		std::vector<OBB> cards; // list of cards
		std::vector<NaniteCard> nanite_cards; // list of nanite cards

		GLScene::Mesh* mesh; // mesh data

		GLuint sdfArray; // sdf array
		GLuint normalArray; // normal array
		GLuint seedArray; // seed array
		GLuint shaderProgram; // shader program

		void initSDF();
		void computeSDF();
		void initCards(float area);
		void sampleComputeCards(float area);
	};
};
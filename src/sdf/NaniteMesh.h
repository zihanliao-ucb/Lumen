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

namespace CGL {

	struct OBB {
		Eigen::Vector3f origin; // min corner of the box
		Eigen::Matrix3f rotation; // local basis: columns are axes
		Eigen::Vector3f extent;   // size along each axis
		std::vector<Eigen::Vector3f> points; // points in this cluster

		bool operator<(const OBB& other) const {
			return (extent.prod() / points.size()) < (other.extent.prod() / other.points.size()); // Larger boxes go to the top
		};
	};
	
	struct NaniteCard {
		OBB box; // bounding box of the card
		int sx, sy, sz; // size of the card
		int bias; // bias of the card
		float vx, vy, vz; // size of each voxel in the card
		float mat[9]; // transformation matrix
		std::vector<float> normals; // normal data
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
		void computeCards(float area);
	};
};
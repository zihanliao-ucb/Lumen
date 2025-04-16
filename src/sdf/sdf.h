#include "CGL/CGL.h"
#include "CGL/color.h"
#include "GL/glew.h"

#include "pathtracer/camera.h"
#include "scene/gl_scene/scene.h"
#include "scene/gl_scene/mesh.h" 

#include <iostream>
#include <vector>
#include <cmath>

namespace CGL {


	class SDFRenderer {
	public:
		SDFRenderer(Camera* camera, GLScene::Scene* scene, GLScene::Mesh* control_mesh);
		~SDFRenderer();
		void render(); // cast ray from camera to sdf
		void setCamera(Camera* cam);
		void setScene(GLScene::Scene* sc);
		void computeSDF(); // compute sdf using JFA
		void moveControlMesh(Vector3D delta); // move control mesh

	private:
		int numSurfacePts;
		int control_pts_offset; // offset of the control mesh points in the surface points buffer
		int control_pts_size; // size of the control mesh points
		Vector3D bbox_min; // min point of the bounding box
		double sdfVoxelLength; // length of each voxel in the sdf
		double sdfSize; // length of the sdf
		Camera* camera;
		GLScene::Mesh* control_mesh;

		GLScene::Scene* scene;
		GLuint surfacePoints;
		GLuint surfaceNormals;
		GLuint sdfTexture; // 3D sdf field stored in a 3D texture
		GLuint normalTexture;
		GLuint screenTexture;
		GLuint quadVAO;

		GLuint divergeShader;
		GLuint seedShader;
		GLuint jfaShader;
		GLuint rayShader;
		GLuint fullscreenShader;
		GLuint translationShader;
	};

};
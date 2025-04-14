#include "CGL/CGL.h"
#include "CGL/color.h"
#include "GL/glew.h"

#include "pathtracer/camera.h"
#include "scene/gl_scene/scene.h"

#include <iostream>
#include <vector>
#include <cmath>

namespace CGL {


	class SDFRenderer {
	public:
		SDFRenderer(Camera* camera, GLScene::Scene* scene);
		~SDFRenderer();
		void render(); // cast ray from camera to sdf
		void setCamera(Camera* cam);
		void setScene(GLScene::Scene* sc);
		void computeSDF(); // compute sdf using JFA

	private:
		int numSurfacePts;
		Vector3D bbox_min; // min point of the bounding box
		double sdfVoxelLength; // length of each voxel in the sdf
		double sdfSize; // length of the sdf
		Camera* camera;

		GLScene::Scene* scene;
		GLuint surfacePoints;
		GLuint surfaceNormals;
		GLuint sdfTexture; // 3D sdf field stored in a 3D texture
		GLuint normalTexture;
		GLuint screenTexture;

		GLuint seedShader;
		GLuint jfaShader;
		GLuint rayShader;
		GLuint fullscreenShader;
		GLuint quadVAO;
	};

};
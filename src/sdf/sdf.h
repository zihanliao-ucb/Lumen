#include "CGL/CGL.h"
#include "CGL/color.h"
#include "GL/glew.h"

#include "pathtracer/camera.h"
#include "scene/gl_scene/scene.h"
#include "scene/gl_scene/mesh.h" 
#include "sdf/NaniteMesh.h"
#include "scene/light.h"

#include <iostream>
#include <vector>
#include <cmath>

namespace CGL {


	class SDFRenderer {
	public:
		SDFRenderer(Camera* camera, GLScene::Scene* scene, GLScene::Mesh* control_mesh);
		~SDFRenderer();
		void lumenUpdate(int numsample);
		void render(); // cast ray from camera to sdf
		void moveControlMesh(Vector3D delta); // move control mesh
		void visualizeCards(); // visualize the cards

	private:
		int control_obj_idx; // offset of the control mesh points in the surface points buffer
		int total_card_voxels; // total number of voxels in the cards
		int num_surface_pts;

		Camera* camera;
		GLScene::Mesh* control_mesh;
		GLScene::Scene* scene;
		std::vector<NaniteMesh> nanite_meshes; // list of nanite meshes

		GLuint sdfSizeArray_g, sdfCoordArray_g, sdfArray_g, objCardArray_g, cardSizeArray_g, cardCoordArray_g, cardSurfaceArray_g, cardPointArray_g, cardRadianceArray_g;

		GLuint screenTexture;
		GLuint quadVAO;

		GLuint rayShader;
		GLuint lumenShader;
		GLuint fullscreenShader;
	};

};
#include "sdf/sdf.h"
#include <fstream>

using namespace CGL;

SDFRenderer::SDFRenderer(Camera* camera, GLScene::Scene* scene, GLScene::Mesh* control_mesh)
  : camera(camera), scene(scene), control_mesh(control_mesh) {

	std::vector<int> sdfSizeArray, cardSizeArray, objCardArray;
	std::vector<float> sdfCoordArray, sdfArray, cardCoordArray, cardSurfaceArray, cardPointArray, cardRadianceArray;
  int sdfBias = 0, cardBias = 0, cardArrayBias = 0;
  for (GLScene::SceneObject* obj : scene->objects) {
    // If the obj is the control_mesh
    if (static_cast<GLScene::Mesh*>(obj) == control_mesh) {
      control_obj_idx = nanite_meshes.size();
    }
		nanite_meshes.emplace_back(static_cast<GLScene::Mesh*>(obj));
		nanite_meshes.back().sdfBias = sdfBias;
		nanite_meshes.back().cardBias = cardBias;
		auto& nm = nanite_meshes.back();
		sdfBias += nm.sx * nm.sy * nm.sz;
		cardBias += nm.nCards;

		sdfSizeArray.push_back(nm.sx);
		sdfSizeArray.push_back(nm.sy);
		sdfSizeArray.push_back(nm.sz);
		sdfSizeArray.push_back(nm.sdfBias);

		sdfCoordArray.push_back(nm.ox);
		sdfCoordArray.push_back(nm.oy);
		sdfCoordArray.push_back(nm.oz);
		for (int i = 0; i < 9; ++i) {
			sdfCoordArray.push_back(nm.mat[i]);
		}

		sdfArray.insert(sdfArray.end(), nm.sdf.begin(), nm.sdf.end());

		objCardArray.push_back(nm.cardBias);
		objCardArray.push_back(nm.nCards);

		Vector3D reflectance = static_cast<DiffuseBSDF*>(nm.mesh->get_bsdf())->reflectance;
		Eigen::Vector3f color = Vector3f_fromCGL(reflectance);
		cardSurfaceArray.insert(cardSurfaceArray.end(), color.data(), color.data() + 3);
		cardSurfaceArray.push_back(1.0); // roughness

		Vector3D emission = nm.mesh->get_bsdf()->get_emission();
		Eigen::Vector3f emissionColor = Vector3f_fromCGL(emission);
		cardSurfaceArray.insert(cardSurfaceArray.end(), emissionColor.data(), emissionColor.data() + 3);

		std::cout << "Card surface: " << color.transpose() << " " << 1.0f << " " << emissionColor.transpose() << std::endl;

		for (int i = 0; i < nm.nCards; ++i) {
			auto& card = nm.nanite_cards[i];
			card.bias += cardArrayBias;
			cardSizeArray.insert(cardSizeArray.end(), card.size.data(), card.size.data() + 3);
			cardSizeArray.push_back(card.bias);

			cardCoordArray.insert(cardCoordArray.end(), card.box.origin.data(), card.box.origin.data() + 3);
			Eigen::Matrix3f M = card.getTransform();
			cardCoordArray.insert(cardCoordArray.end(), M.data(), M.data() + 9);

			std::vector<float> cardPointData = card.getPointData();
			cardPointArray.insert(cardPointArray.end(), cardPointData.begin(), cardPointData.end());

		}
		cardArrayBias += nm.nCardsVoxels;
  }
	total_card_voxels = cardArrayBias;

	cardRadianceArray.resize(total_card_voxels * 64 * 3, 0.0f);

	std::cout << "Total card size: " << total_card_voxels << std::endl;
	std::cout << "Screen size: " << camera->screen_width() * camera->screen_height() << std::endl;

	// Create the sdf size array
	glGenBuffers(1, &sdfSizeArray_g);
	glBindBuffer(GL_ARRAY_BUFFER, sdfSizeArray_g);
	glBufferData(GL_ARRAY_BUFFER, sdfSizeArray.size() * sizeof(int), sdfSizeArray.data(), GL_STATIC_DRAW);
	// Create the sdf coord array
	glGenBuffers(1, &sdfCoordArray_g);
	glBindBuffer(GL_ARRAY_BUFFER, sdfCoordArray_g);
	glBufferData(GL_ARRAY_BUFFER, sdfCoordArray.size() * sizeof(float), sdfCoordArray.data(), GL_STATIC_DRAW);
	// Create the sdf array
	glGenBuffers(1, &sdfArray_g);
	glBindBuffer(GL_ARRAY_BUFFER, sdfArray_g);
	glBufferData(GL_ARRAY_BUFFER, sdfArray.size() * sizeof(float), sdfArray.data(), GL_STATIC_DRAW);
	// Create the obj card array
	glGenBuffers(1, &objCardArray_g);
	glBindBuffer(GL_ARRAY_BUFFER, objCardArray_g);
	glBufferData(GL_ARRAY_BUFFER, objCardArray.size() * sizeof(int), objCardArray.data(), GL_STATIC_DRAW);
	// Create the card size array
	glGenBuffers(1, &cardSizeArray_g);
	glBindBuffer(GL_ARRAY_BUFFER, cardSizeArray_g);
	glBufferData(GL_ARRAY_BUFFER, cardSizeArray.size() * sizeof(int), cardSizeArray.data(), GL_STATIC_DRAW);
	// Create the card coord array
	glGenBuffers(1, &cardCoordArray_g);
	glBindBuffer(GL_ARRAY_BUFFER, cardCoordArray_g);
	glBufferData(GL_ARRAY_BUFFER, cardCoordArray.size() * sizeof(float), cardCoordArray.data(), GL_STATIC_DRAW);
	// Create the card surface array
	glGenBuffers(1, &cardSurfaceArray_g);
	glBindBuffer(GL_ARRAY_BUFFER, cardSurfaceArray_g);
	glBufferData(GL_ARRAY_BUFFER, cardSurfaceArray.size() * sizeof(float), cardSurfaceArray.data(), GL_STATIC_DRAW);
	// Create the card points array
	glGenBuffers(1, &cardPointArray_g);
	glBindBuffer(GL_ARRAY_BUFFER, cardPointArray_g);
	glBufferData(GL_ARRAY_BUFFER, cardPointArray.size() * sizeof(float), cardPointArray.data(), GL_STATIC_DRAW);
	// Create the card radiance array
	glGenBuffers(1, &cardRadianceArray_g);
	glBindBuffer(GL_ARRAY_BUFFER, cardRadianceArray_g);
	glBufferData(GL_ARRAY_BUFFER, cardRadianceArray.size() * sizeof(float), cardRadianceArray.data(), GL_STATIC_DRAW);

	// Create the screen texture
	glGenTextures(1, &screenTexture);
	glBindTexture(GL_TEXTURE_2D, screenTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, camera->screen_width(), camera->screen_height(), 0, GL_RGBA, GL_FLOAT, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// Create fullscreen quad
	glGenVertexArrays(1, &quadVAO);
	glGenVertexArrays(1, &quadVAO);
	glBindVertexArray(quadVAO);

	// Compile and link the Lumen shader
	lumenShader = glCreateProgram();
	GLuint cs_lumen = glCreateShader(GL_COMPUTE_SHADER);
	std::ifstream file_lumen("D:/UCB 25Spring/CS 284A/Lumen/src/shaders/lumen.comp");
	std::string code_lumen((std::istreambuf_iterator<char>(file_lumen)), std::istreambuf_iterator<char>());
	const char* src_lumen = code_lumen.c_str();
	glShaderSource(cs_lumen, 1, &src_lumen, NULL);
	glCompileShader(cs_lumen);
	glAttachShader(lumenShader, cs_lumen);
	glLinkProgram(lumenShader);
	GLint lumenLinked = 0;
	glGetProgramiv(lumenShader, GL_LINK_STATUS, &lumenLinked);
	if (!lumenLinked) {
		char infoLog[512];
		glGetProgramInfoLog(lumenShader, 512, NULL, infoLog);
		std::cerr << "Lumen shader linking failed:\n" << infoLog << std::endl;
	}
	glDeleteShader(cs_lumen);

	// Compile and link the ray shader
	rayShader = glCreateProgram();
	GLuint cs = glCreateShader(GL_COMPUTE_SHADER);
	std::ifstream file("D:/UCB 25Spring/CS 284A/Lumen/src/shaders/raymarch.comp");
	std::string code((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
	const char* src = code.c_str();
	glShaderSource(cs, 1, &src, NULL);
	glCompileShader(cs);
	glAttachShader(rayShader, cs);
	glLinkProgram(rayShader);
	GLint rayLinked = 0;
	glGetProgramiv(rayShader, GL_LINK_STATUS, &rayLinked);
	if (!rayLinked) {
		char infoLog[512];
		glGetProgramInfoLog(rayShader, 512, NULL, infoLog);
		std::cerr << "Ray shader linking failed:\n" << infoLog << std::endl;
	}
	glDeleteShader(cs);

	// Compile fullscreen shader
	GLuint vs = glCreateShader(GL_VERTEX_SHADER);
	GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
	std::ifstream vfile("D:/UCB 25Spring/CS 284A/Lumen/src/shaders/fullscreen.vert");
	std::ifstream ffile("D:/UCB 25Spring/CS 284A/Lumen/src/shaders/fullscreen.frag");
	std::string vcode((std::istreambuf_iterator<char>(vfile)), std::istreambuf_iterator<char>());
	std::string fcode((std::istreambuf_iterator<char>(ffile)), std::istreambuf_iterator<char>());
	const char* vsrc = vcode.c_str();
	const char* fsrc = fcode.c_str();
	glShaderSource(vs, 1, &vsrc, NULL);
	glShaderSource(fs, 1, &fsrc, NULL);
	glCompileShader(vs);
	glCompileShader(fs);
	fullscreenShader = glCreateProgram();
	glAttachShader(fullscreenShader, vs);
	glAttachShader(fullscreenShader, fs);
	glLinkProgram(fullscreenShader);
	GLint fullscreenLinked = 0;
	glGetProgramiv(fullscreenShader, GL_LINK_STATUS, &fullscreenLinked);
	if (!fullscreenLinked) {
		char infoLog[512];
		glGetProgramInfoLog(fullscreenShader, 512, NULL, infoLog);
		std::cerr << "Fullscreen shader linking failed:\n" << infoLog << std::endl;
	}
	glDeleteShader(vs);
	glDeleteShader(fs);
}

SDFRenderer::~SDFRenderer() {
	// Delete the sdf arrays
	glDeleteBuffers(1, &sdfSizeArray_g);
	glDeleteBuffers(1, &sdfCoordArray_g);
	glDeleteBuffers(1, &sdfArray_g);
	// Delete the screen texture
	glDeleteTextures(1, &screenTexture);
	// Delete the fullscreen quad
	glDeleteVertexArrays(1, &quadVAO);
	// Delete the shaders
	glDeleteProgram(rayShader);
	glDeleteProgram(fullscreenShader);
}

void SDFRenderer::lumenUpdate(int numsample) {
	// Use the Lumen shader
	glUseProgram(lumenShader);
	// Bind the arrays
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, sdfSizeArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, sdfCoordArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, sdfArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, objCardArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, cardSizeArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, cardCoordArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, cardSurfaceArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, cardPointArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 8, cardRadianceArray_g);
	// Upload uniforms
	glUniform1i(glGetUniformLocation(lumenShader, "uNumSurfacePoints"), num_surface_pts);
	glUniform1i(glGetUniformLocation(lumenShader, "uNumObjects"), nanite_meshes.size());
	glUniform1i(glGetUniformLocation(lumenShader, "seed"), static_cast<int>(time(nullptr)));
	glUniform1i(glGetUniformLocation(lumenShader, "uNumSample"), numsample);
	// Dispatch compute shader
	int groups_x = (total_card_voxels + 63) / 64;
	glDispatchCompute(groups_x, 1, 1);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT); // ensure writes are visible
}

void SDFRenderer::render() {
	// Use the ray shader
	glUseProgram(rayShader);
	// Bind the screen texture
	glBindImageTexture(0, screenTexture, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
	// Bind the arrays
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, sdfSizeArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, sdfCoordArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, sdfArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, objCardArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, cardSizeArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, cardCoordArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, cardSurfaceArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 8, cardPointArray_g);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 9, cardRadianceArray_g);
	
	// Camera and screen parameters
	Vector3D camPos = camera->position();
	Vector3D view = (camera->view_point() - camPos).unit();
	Vector3D up = camera->up_dir().unit();
	Vector3D right = cross(view, up).unit();
	float c2w_flat[9] = {
		static_cast<float>(right[0]), static_cast<float>(right[1]), static_cast<float>(right[2]),
		static_cast<float>(up[0]), static_cast<float>(up[1]), static_cast<float>(up[2]),
		static_cast<float>(-view[0]), static_cast<float>(-view[1]), static_cast<float>(-view[2])
	};
	// Upload uniforms
	glUniform3f(glGetUniformLocation(rayShader, "uCameraPos"), static_cast<float>(camPos.x), static_cast<float>(camPos.y), static_cast<float>(camPos.z));
	glUniformMatrix3fv(glGetUniformLocation(rayShader, "uC2WMatrix"), 1, GL_FALSE, c2w_flat);
	glUniform1i(glGetUniformLocation(rayShader, "uScreenW"), camera->screen_width());
	glUniform1i(glGetUniformLocation(rayShader, "uScreenH"), camera->screen_height());
	float tanFovY = tanf(0.5f * radians(camera->v_fov()));
	glUniform1f(glGetUniformLocation(rayShader, "uTanFovY"), tanFovY);
	glUniform1f(glGetUniformLocation(rayShader, "uAspect"), static_cast<float>(camera->aspect_ratio()));
	glUniform1i(glGetUniformLocation(rayShader, "uNumObjects"), nanite_meshes.size());

	// Dispatch compute shader
	int groups_x = (camera->screen_width() + 7) / 8;
	int groups_y = (camera->screen_height() + 7) / 8;
	glDispatchCompute(groups_x, groups_y, 1);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT); // ensure writes are visible

  // Draw fullscreen quad
	glClear(GL_COLOR_BUFFER_BIT);
  glUseProgram(fullscreenShader);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, screenTexture);
  glUniform1i(glGetUniformLocation(fullscreenShader, "screenImage"), 0);
  glBindVertexArray(quadVAO);
  glDrawArrays(GL_TRIANGLES, 0, 3);
}

void SDFRenderer::visualizeCards() {
	// Visualize the Cards
	// Switch back to 3D camera projection
	glUseProgram(0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(camera->v_fov(), camera->aspect_ratio(), 0.1, 1000.0); // near/far planes

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	Vector3D eye = camera->position();
	Vector3D center = camera->view_point();
	Vector3D up = camera->up_dir();
	gluLookAt(eye.x, eye.y, eye.z, center.x, center.y, center.z, up.x, up.y, up.z);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glLineWidth(1.0f);              // make lines visible
	for (int i = 0; i < nanite_meshes.size(); ++i) {
		const auto& nanite_cards = nanite_meshes[i].nanite_cards;
		for (const auto& card : nanite_cards) {
			const Eigen::Vector3f& o = card.box.origin;
			const Eigen::Matrix3f& r = card.box.rotation;
			const Eigen::Vector3f& e = card.box.extent;

			// Basis-scaled axes
			Eigen::Vector3f x = r.col(0) * e.x();
			Eigen::Vector3f y = r.col(1) * e.y();
			Eigen::Vector3f z = r.col(2) * e.z();

			// Compute corners
			Eigen::Vector3f c0 = o;
			Eigen::Vector3f c1 = o + x;
			Eigen::Vector3f c2 = o + y;
			Eigen::Vector3f c3 = o + z;
			Eigen::Vector3f c4 = o + x + y;
			Eigen::Vector3f c5 = o + x + z;
			Eigen::Vector3f c6 = o + y + z;
			Eigen::Vector3f c7 = o + x + y + z;

			glColor3f(1.0f, 0.0f, 0.0f);
			glBegin(GL_LINES);

			// Bottom square
			glVertex3fv(c0.data()); glVertex3fv(c1.data());
			glVertex3fv(c0.data()); glVertex3fv(c2.data());
			glVertex3fv(c1.data()); glVertex3fv(c4.data());
			glVertex3fv(c2.data()); glVertex3fv(c4.data());

			// Top square
			glVertex3fv(c3.data()); glVertex3fv(c5.data());
			glVertex3fv(c3.data()); glVertex3fv(c6.data());
			glVertex3fv(c5.data()); glVertex3fv(c7.data());
			glVertex3fv(c6.data()); glVertex3fv(c7.data());

			// Vertical edges
			glVertex3fv(c0.data()); glVertex3fv(c3.data());
			glVertex3fv(c1.data()); glVertex3fv(c5.data());
			glVertex3fv(c2.data()); glVertex3fv(c6.data());
			glVertex3fv(c4.data()); glVertex3fv(c7.data());

			glEnd();
		}
	}

}

void SDFRenderer::moveControlMesh(Vector3D delta) {
	// Move the origin of the control mesh sdf coordinate
	NaniteMesh& control = nanite_meshes[control_obj_idx];
	control.ox += static_cast<float>(delta.x);
	control.oy += static_cast<float>(delta.y);
	control.oz += static_cast<float>(delta.z);

	// Update the sdfCoordArray_g at [control_obj_idx * 12 : control_obj_idx * 12 + 3]
	float new_origin[3] = {
		control.ox,
		control.oy,
		control.oz
	};

	glBindBuffer(GL_SHADER_STORAGE_BUFFER, sdfCoordArray_g);
	glBufferSubData(GL_SHADER_STORAGE_BUFFER,
		control_obj_idx * 12 * sizeof(float),
		3 * sizeof(float),
		new_origin);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

	// Move the cards
	for (auto& card : control.nanite_cards) {
		card.box.origin += Eigen::Vector3f(delta.x, delta.y, delta.z);
	}

	// for all cards from objcardArray_g[control_obj_idx * 2] with length objcardArray_g[control_obj_idx * 2 + 1]
	// Update the cardCoordArray_g at [control_obj_idx * 12 + 0 : control_obj_idx * 12 + 3]
	for (int i = 0; i < control.nCards; ++i) {
		auto& card = control.nanite_cards[i];
		float new_card_origin[3] = {
			card.box.origin.x(),
			card.box.origin.y(),
			card.box.origin.z()
		};
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, cardCoordArray_g);
		glBufferSubData(GL_SHADER_STORAGE_BUFFER,
			(control.cardBias + i) * 12 * sizeof(float),
			3 * sizeof(float),
			new_card_origin);
	}
}

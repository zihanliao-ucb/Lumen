#include "sdf/sdf.h"
#include <fstream>

using namespace CGL;

SDFRenderer::SDFRenderer(Camera* camera, GLScene::Scene* scene, GLScene::Mesh* control_mesh)
  : camera(camera), scene(scene), control_mesh(control_mesh), surfacePoints(0), sdfTexture(0), seedShader(0), jfaShader(0) {

  sdfSize = 128;
	std::cout << "SDF size: " << sdfSize << std::endl;

  glGenTextures(1, &sdfTexture);
  glBindTexture(GL_TEXTURE_3D, sdfTexture);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

  double sdfLength = scene->get_bbox().max.x - scene->get_bbox().min.x;
  sdfLength = std::max(sdfLength, scene->get_bbox().max.y - scene->get_bbox().min.y);
  sdfLength = std::max(sdfLength, scene->get_bbox().max.z - scene->get_bbox().min.z);
  bbox_min = scene->get_bbox().min - 0.05 * sdfLength;
	sdfLength *= 1.1; // add some padding
  sdfVoxelLength = sdfLength / sdfSize;
	std::cout << "SDF voxel length: " << sdfVoxelLength << std::endl;

  int numVoxels = sdfSize * sdfSize * sdfSize;
  std::vector<float> sdfInitsSDF(numVoxels * 4, 5.0f);
  glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA32F, sdfSize, sdfSize, sdfSize, 0,
    GL_RGBA, GL_FLOAT, sdfInitsSDF.data());

  glGenTextures(1, &normalTexture);
  glBindTexture(GL_TEXTURE_3D, normalTexture);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
  std::vector<float> sdfInitNormal(numVoxels * 4, 0.0f);
  glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA32F, sdfSize, sdfSize, sdfSize, 0,
    GL_RGBA, GL_FLOAT, sdfInitNormal.data());

	// Compile diverge shader
	divergeShader = glCreateProgram();
	GLuint divergeCS = glCreateShader(GL_COMPUTE_SHADER);
	std::ifstream file3("D:/UCB 25Spring/CS 284A/Lumen/src/shaders/jfa_diverge.comp");
	std::string code3((std::istreambuf_iterator<char>(file3)), std::istreambuf_iterator<char>());
	const char* src3 = code3.c_str();
	glShaderSource(divergeCS, 1, &src3, NULL);
	glCompileShader(divergeCS);
  glAttachShader(divergeShader, divergeCS);
  glLinkProgram(divergeShader);
	GLint success3 = 0;
  glGetProgramiv(divergeShader, GL_LINK_STATUS, &success3);
	if (!success3) {
		char infoLog[512];
		glGetShaderInfoLog(divergeShader, 512, NULL, infoLog);
		std::cerr << "Diverge shader linking failed:\n" << infoLog << std::endl;
	}
	glDeleteShader(divergeCS);

  // Compile seed shader
  seedShader = glCreateProgram();
  GLuint seedCS = glCreateShader(GL_COMPUTE_SHADER);
  std::ifstream file1("D:/UCB 25Spring/CS 284A/Lumen/src/shaders/jfa_step.comp");
  std::string code1((std::istreambuf_iterator<char>(file1)), std::istreambuf_iterator<char>());
  const char* src1 = code1.c_str();
  glShaderSource(seedCS, 1, &src1, NULL);
  glCompileShader(seedCS);

  GLint success = 0;
  glGetShaderiv(seedCS, GL_COMPILE_STATUS, &success);
  if (!success) {
    char infoLog[512];
    glGetShaderInfoLog(seedCS, 512, NULL, infoLog);
    std::cerr << "Seed shader compilation failed:\n" << infoLog << std::endl;
  }

  glAttachShader(seedShader, seedCS);
  glLinkProgram(seedShader);

  GLint linked = 0;
  glGetProgramiv(seedShader, GL_LINK_STATUS, &linked);
  if (!linked) {
    char infoLog[512];
    glGetProgramInfoLog(seedShader, 512, NULL, infoLog);
    std::cerr << "Seed shader linking failed:\n" << infoLog << std::endl;
  }

  glDeleteShader(seedCS);

  // Compile JFA propagation shader
  jfaShader = glCreateProgram();
  GLuint jfaCS = glCreateShader(GL_COMPUTE_SHADER);
  std::ifstream file2("D:/UCB 25Spring/CS 284A/Lumen/src/shaders/jfa_propagation.comp");
  std::string code2((std::istreambuf_iterator<char>(file2)), std::istreambuf_iterator<char>());
  const char* src2 = code2.c_str();
  glShaderSource(jfaCS, 1, &src2, NULL);
  glCompileShader(jfaCS);
  glAttachShader(jfaShader, jfaCS);
  glLinkProgram(jfaShader);
	GLint jfaLinked = 0;
	glGetProgramiv(jfaShader, GL_LINK_STATUS, &jfaLinked);
	if (!jfaLinked) {
		char infoLog[512];
		glGetProgramInfoLog(jfaShader, 512, NULL, infoLog);
		std::cerr << "JFA shader linking failed:\n" << infoLog << std::endl;
	}
  glDeleteShader(jfaCS);

	// Create buffer for surface points and normals
  double pixelArea = sdfVoxelLength * sdfVoxelLength;
  glGenBuffers(1, &surfacePoints);
	glGenBuffers(1, &surfaceNormals);
  std::vector<std::pair<Vector3D, Vector3D>> surface_pts;
  for (GLScene::SceneObject* obj : scene->objects) {
    std::vector<std::pair<Vector3D, Vector3D>> pts = obj->sample_points(pixelArea);
    // If the obj is the control_mesh
    if (static_cast<GLScene::Mesh*>(obj) == control_mesh) {
			control_pts_offset = surface_pts.size();
			control_pts_size = pts.size();
			std::cout << "Control mesh found, "
				<< "control_pts_offset: " << control_pts_offset
				<< ", control_pts_size: " << control_pts_size << std::endl;
    }
    surface_pts.insert(surface_pts.end(), pts.begin(), pts.end());
  }
	numSurfacePts = surface_pts.size();
  // Upload surface points
  std::vector<float> surface_pts_f;
  std::vector<float> surface_nms_f;
  surface_pts_f.reserve(surface_pts.size() * 4);
  for (const auto& p : surface_pts) {
    surface_pts_f.push_back(static_cast<float>(p.first.x));
    surface_pts_f.push_back(static_cast<float>(p.first.y));
    surface_pts_f.push_back(static_cast<float>(p.first.z));
    surface_pts_f.push_back(0.0f); // padding

		surface_nms_f.push_back(static_cast<float>(p.second.x));
		surface_nms_f.push_back(static_cast<float>(p.second.y));
		surface_nms_f.push_back(static_cast<float>(p.second.z));
		surface_nms_f.push_back(0.0f); // padding
  }
  glBindBuffer(GL_SHADER_STORAGE_BUFFER, surfacePoints);
  glBufferData(GL_SHADER_STORAGE_BUFFER, surface_pts_f.size() * sizeof(float), surface_pts_f.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, surfaceNormals);
	glBufferData(GL_SHADER_STORAGE_BUFFER, surface_nms_f.size() * sizeof(float), surface_nms_f.data(), GL_STATIC_DRAW);

	// Compile ray shader
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

  // Comile control_mesh translation shader
	translationShader = glCreateProgram();
	GLuint translationCS = glCreateShader(GL_COMPUTE_SHADER);
	std::ifstream translationFile("D:/UCB 25Spring/CS 284A/Lumen/src/shaders/translation.comp");
	std::string translationCode((std::istreambuf_iterator<char>(translationFile)), std::istreambuf_iterator<char>());
	const char* translationSrc = translationCode.c_str();
	glShaderSource(translationCS, 1, &translationSrc, NULL);
	glCompileShader(translationCS);
	glAttachShader(translationShader, translationCS);
	glLinkProgram(translationShader);
	GLint translationLinked = 0;
	glGetProgramiv(translationShader, GL_LINK_STATUS, &translationLinked);
	if (!translationLinked) {
		char infoLog[512];
		glGetProgramInfoLog(translationShader, 512, NULL, infoLog);
		std::cerr << "Translation shader linking failed:\n" << infoLog << std::endl;
	}
	glDeleteShader(translationCS);

	// Create fullscreen quad
  glGenVertexArrays(1, &quadVAO);
  glGenVertexArrays(1, &quadVAO);
  glBindVertexArray(quadVAO);

  // Allocate screen texture
  glGenTextures(1, &screenTexture);
  glBindTexture(GL_TEXTURE_2D, screenTexture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, camera->screen_width(), camera->screen_height(), 0, GL_RGBA, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

SDFRenderer::~SDFRenderer() {
  glDeleteBuffers(1, &surfacePoints);
  glDeleteTextures(1, &sdfTexture);
}

void SDFRenderer::setCamera(Camera* cam) {
  camera = cam;
}

void SDFRenderer::setScene(GLScene::Scene* sc) {
  scene = sc;
}

void SDFRenderer::moveControlMesh(Vector3D delta) {
	// Move control mesh points
	glUseProgram(translationShader);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, surfacePoints);
	glUniform1i(glGetUniformLocation(translationShader, "uOffset"), control_pts_offset);
	glUniform1i(glGetUniformLocation(translationShader, "uSize"), control_pts_size);
  glUniform3f(glGetUniformLocation(translationShader, "uDelta"),
    static_cast<float>(delta.x),
    static_cast<float>(delta.y),
    static_cast<float>(delta.z));
	glDispatchCompute(numSurfacePts / 64 + 1, 1, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
  // Add all sdf value by sdfVoxelLength / 2
  glUseProgram(divergeShader);
  glBindImageTexture(0, sdfTexture, 0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA32F);
  glUniform1f(glGetUniformLocation(divergeShader, "uVoxelSize"), static_cast<float>(sdfVoxelLength));
  glUniform1i(glGetUniformLocation(divergeShader, "uSdfSize"), sdfSize);
  glDispatchCompute(sdfSize / 8, sdfSize / 8, sdfSize / 8);
  glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

void SDFRenderer::computeSDF() {
  // Seed pass
  glUseProgram(seedShader);
  glBindImageTexture(0, sdfTexture, 0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA32F);
	glBindImageTexture(1, normalTexture, 0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA32F);
  glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, surfacePoints);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, surfaceNormals);

  glUniform3f(glGetUniformLocation(seedShader, "uBoundingBoxOrigin"),
    static_cast<float>(bbox_min.x),
    static_cast<float>(bbox_min.y),
    static_cast<float>(bbox_min.z));
  glUniform1f(glGetUniformLocation(seedShader, "uVoxelSize"), static_cast<float>(sdfVoxelLength));
  glUniform1i(glGetUniformLocation(seedShader, "uSdfSize"), sdfSize);

  glDispatchCompute(numSurfacePts / 64 + 1, 1, 1);
  glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

  // JFA propagation passes
  glUseProgram(jfaShader);
  glBindImageTexture(0, sdfTexture, 0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA32F);
	glBindImageTexture(1, normalTexture, 0, GL_TRUE, 0, GL_READ_WRITE, GL_RGBA32F);
  glUniform3f(glGetUniformLocation(seedShader, "uBoundingBoxOrigin"),
    static_cast<float>(bbox_min.x),
    static_cast<float>(bbox_min.y),
    static_cast<float>(bbox_min.z));
  glUniform1f(glGetUniformLocation(jfaShader, "uVoxelSize"), (float)sdfVoxelLength);
  glUniform1i(glGetUniformLocation(jfaShader, "uSdfSize"), sdfSize);
  glDispatchCompute(sdfSize / 8, sdfSize / 8, sdfSize / 8);
  glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

	//std::cout << "SDF computed" << std::endl;

	// Plot SDF points
 // std::vector<float> sdfCPU(sdfSize * sdfSize * sdfSize * 4);
 // glBindTexture(GL_TEXTURE_3D, sdfTexture);
 // glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_FLOAT, sdfCPU.data());
 // glPointSize(3.0f);
	//glDisable(GL_LIGHTING);
 // glBegin(GL_POINTS);
 // glColor3f(1.0f, 0.0f, 0.0f);
 // for (int z = 0; z < sdfSize; ++z) {
 //   for (int y = 0; y < sdfSize; ++y) {
 //     for (int x = 0; x < sdfSize; ++x) {
 //       int idx = x + y * sdfSize + z * sdfSize * sdfSize;
 //       float dist = sdfCPU[idx * 4 + 3];
 //       if (abs(dist) < 0.001f) {
 //         Vector3D p_world = bbox_min +
 //           Vector3D((x + 0.5f) * sdfVoxelLength,
 //             (y + 0.5f) * sdfVoxelLength,
 //             (z + 0.5f) * sdfVoxelLength);
 //         glVertex3d(p_world.x, p_world.y, p_world.z);
 //       }
 //     }
 //   }
 // }
 // glEnd();
	//std::cout << "SDF points count: " << cnt << std::endl;
}


void SDFRenderer::render() {
  if (!camera || !scene) return;

  // This function should cast rays from the camera into the SDF field
  glUseProgram(rayShader);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_3D, sdfTexture);
  glUniform1i(glGetUniformLocation(rayShader, "sdfTexture"), 0);
  glBindImageTexture(1, screenTexture, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);

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
  glUniform3f(glGetUniformLocation(rayShader, "uBoundingBoxOrigin"), static_cast<float>(bbox_min.x), static_cast<float>(bbox_min.y), static_cast<float>(bbox_min.z));
  glUniform1f(glGetUniformLocation(rayShader, "uVoxelSize"), static_cast<float>(sdfVoxelLength));
  glUniform1i(glGetUniformLocation(rayShader, "uSdfSize"), sdfSize);
  glUniform1i(glGetUniformLocation(rayShader, "uScreenW"), camera->screen_width());
  glUniform1i(glGetUniformLocation(rayShader, "uScreenH"), camera->screen_height());
  float tanFovY = tanf(0.5f * radians(camera->v_fov()));
  glUniform1f(glGetUniformLocation(rayShader, "uTanFovY"), tanFovY);
  glUniform1f(glGetUniformLocation(rayShader, "uAspect"), static_cast<float>(camera->aspect_ratio()));

  glDispatchCompute((camera->screen_width() + 7) / 8, (camera->screen_height() + 7) / 8, 1);
  glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

	// Step 2: Download screen texture from GPU
	//std::vector<float> screenCPU(camera->screen_width() * camera->screen_height() * 4);
	//glBindTexture(GL_TEXTURE_2D, screenTexture);
	//glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, screenCPU.data());
	//float hitmax = 0.0f;
	//for (int i = 0; i < camera->screen_width() * camera->screen_height(); ++i) {
	//	hitmax = std::max(hitmax, screenCPU[i * 4 + 0]);
	//}
	//for (int i = 0; i < camera->screen_width() * camera->screen_height(); ++i) {
	//	screenCPU[i * 4 + 0] /= hitmax;
	//	screenCPU[i * 4 + 1] /= hitmax;
	//	screenCPU[i * 4 + 2] /= hitmax;
	//	screenCPU[i * 4 + 3] = 1.0f;
	//}
 // // Use glDrawPixels
 // int screenW = camera->screen_width();
 // int screenH = camera->screen_height();
 // glDrawPixels(screenW, screenH, GL_RGBA, GL_FLOAT, screenCPU.data());

  // Draw fullscreen quad
  glClear(GL_COLOR_BUFFER_BIT);
  glUseProgram(fullscreenShader);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, screenTexture);
  glUniform1i(glGetUniformLocation(fullscreenShader, "screenImage"), 0);
  glBindVertexArray(quadVAO);
  glDrawArrays(GL_TRIANGLES, 0, 3);
}

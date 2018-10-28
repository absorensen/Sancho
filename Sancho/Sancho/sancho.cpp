#include "sancho.h"

// settings
Settings settings;
const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 800;
const float Z_NEAR = 0.1f;
const float Z_FAR = 10000.0f;
const float ASPECT_RATIO = ((float)SCR_WIDTH) / SCR_HEIGHT;
static const float SCALE = 1.0f / static_cast<float>(SCR_WIDTH);
static const float PI = 3.14159265359f;
static const double key_press_threshold = 0.25;
float _point_size = 0.01f;
bool octree_show_all_levels = false;
int octree_show_level = 0;
double time_since_last_frame = 0.0;
bool draw_patch_normals = false;
bool draw_patch_planes = false;

// camera
//Camera camera(-2.0f, 1.5f, 3.4f, 0.0f, 1.0f, 0.0f, -50.0f, -12.0f);
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;
float height_of_near_plane;

// timing
double deltaTime = 0.0;
double lastFrame = 0.0;

int main(int argc, char * argv[]) {
	// initialize and configure
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Sancho", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);


	// load OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// configure global opengl state
	// -----------------------------
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_PROGRAM_POINT_SIZE);
	glEnable(GL_POINT_SMOOTH);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glDepthFunc(GL_LESS);
	fprintf(stderr, "OpenGL %s\n", glGetString(GL_VERSION));

	PointCloud point_cloud;
	//load_point_cloud("../Cube.txt", 8, 3, point_cloud);
	load_point_cloud("../stan.txt", 2503, 3, point_cloud);
	//load_point_cloud("../Room2.txt", 112586, 3, point_cloud);
	//load_point_cloud("../Room.txt", 831159, 3, point_cloud);
	//load_point_cloud("../Box.txt", 964806, 3, point_cloud);

	Shader point_cloud_shader("point_cloud.vert","point_cloud.frag");

	GLuint VAO,VBO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat)*point_cloud.size, point_cloud.points, GL_STATIC_DRAW);
	GLint position_attribute = glGetAttribLocation(point_cloud_shader.ID, "aPos");
	glVertexAttribPointer(position_attribute, point_cloud.no_of_coords, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	float fovy = 60; // degrees
	int viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	height_of_near_plane = (float)abs(viewport[3] - viewport[1]) /
		(2 * tan(0.5*fovy*PI / 180.0));

	point_cloud_shader.use();
	point_cloud_shader.setVec3("color", 0.9f, 0.4f, 0.0f);
	point_cloud_shader.setFloat("point_size", _point_size);
	point_cloud_shader.setFloat("z_near", Z_NEAR);
	point_cloud_shader.setFloat("z_far", Z_FAR);
	point_cloud_shader.setFloat("height_of_near_plane", height_of_near_plane);

	// Real-Time Point Cloud Compression begin
	EigenTree eigen_tree;
	Octree tree;
	Shader cube_shader("point_cloud.vert", "point_cloud.frag");
	Shader point_shader("point_cloud.vert", "point_cloud.frag");
	Shader normals_shader("point_cloud.vert", "point_cloud.frag");

	settings.point_shader = &point_shader;
	settings.cube_shader = &cube_shader;
	settings.normals_shader = &normals_shader;
	settings.Z_NEAR = 0.1f;
	settings.Z_FAR = 10000.0f;
	settings.point_size = _point_size;
	settings.height_of_near_plane = height_of_near_plane;
	settings.camera = &camera;
	settings.ASPECT_RATIO = ASPECT_RATIO;
	settings.SCR_HEIGHT = SCR_HEIGHT;
	settings.SCR_WIDTH = SCR_WIDTH;
	settings.draw_patch_normals = &draw_patch_normals;
	settings.draw_patch_planes = &draw_patch_planes;

	bool eigentree_path = false;
	if (eigentree_path) {
		real_time_point_cloud_compression_eigentree(point_cloud, eigen_tree, settings);
	}
	else {
		real_time_point_cloud_compression(point_cloud, tree, settings);
	}

	//Eigen::Vector4d middle(10.0, 12.0, -15.0, 1.0);
	// Real-Time Point Cloud Compression end
	
	Timer t, totalTime;
	while (!glfwWindowShouldClose(window))
	{
		process_input(window);

		const double currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		glClearColor(0.1f, 0.1f, 0.4f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, Z_NEAR, Z_FAR);
		//glm::mat4 view = camera.GetViewMatrix();
		//glm::mat4 model = glm::mat4(1.0f);
		//glm::mat4 MVPmatrix = projection * view * model;

		//point_cloud_shader.use();
		//point_cloud_shader.setMat4("mvp_matrix", MVPmatrix);
		//point_cloud_shader.setVec3("cam_pos", camera.Position);
		//point_cloud_shader.setFloat("point_size", _point_size);

		//glBindVertexArray(VAO);
		//glDrawArrays(GL_POINTS, 0, point_cloud.no_of_points);

		if (eigentree_path) {
			if (octree_show_all_levels) eigen_tree.show();
			else eigen_tree.show(octree_show_level);
		}
		else {
			if (draw_patch_normals) tree.show_normals(0.01f);
			if (octree_show_all_levels) tree.show_tree();
			else tree.show_level(octree_show_level);
		}
		
		
		glBindVertexArray(0);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}   
	//needs a safe deletion for tree structure

	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);

	
	glfwTerminate();
	
	return EXIT_SUCCESS;
}

double deltaTimeMod;
void process_input(GLFWwindow* window) {
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, true);
		return;
	}
	double timeDifference = glfwGetTime() - time_since_last_frame;

	deltaTimeMod = deltaTime;
	
	if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
		deltaTimeMod *= 5.0;
	}

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
		camera.ProcessKeyboard(FORWARD, deltaTimeMod);
	}

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
		camera.ProcessKeyboard(FORWARD, deltaTimeMod);
	}
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
	{
		camera.ProcessKeyboard(BACKWARD, deltaTimeMod);
	}
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
	{
		camera.ProcessKeyboard(LEFT, deltaTimeMod);
	}
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
	{
		camera.ProcessKeyboard(RIGHT, deltaTimeMod);
	}
	if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
	{
		camera.ProcessKeyboard(UP, deltaTimeMod);
	}
	if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
	{
		camera.ProcessKeyboard(DOWN, deltaTimeMod);
	}
	if (timeDifference > key_press_threshold && glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS)
	{
		draw_patch_normals = !draw_patch_normals;
		std::cout << "Draw patch normals equals: " << draw_patch_normals << std::endl;
		time_since_last_frame = glfwGetTime();
	}
	if (timeDifference > key_press_threshold && glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
	{
		draw_patch_planes = !draw_patch_planes;
		std::cout << "Draw patch planes equals: " << draw_patch_planes << std::endl;
		time_since_last_frame = glfwGetTime();
	}
	if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
	{
		_point_size -= 0.01f * deltaTimeMod;
		settings.point_size -= 0.01f * deltaTimeMod;
	}
	if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS)
	{
		_point_size += 0.01f * deltaTimeMod;
		settings.point_size += 0.01f * deltaTimeMod;
		
	}
	if (timeDifference > key_press_threshold && glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS)
	{
		octree_show_all_levels = !octree_show_all_levels;
		time_since_last_frame = glfwGetTime();
	}
	if (timeDifference > key_press_threshold && glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS)
	{
		++octree_show_level;
		std::cout << "Showing octree level: " << octree_show_level << std::endl;
		time_since_last_frame = glfwGetTime();
	}
	if (timeDifference > key_press_threshold && glfwGetKey(window, GLFW_KEY_9) == GLFW_PRESS)
	{
		--octree_show_level;
		std::cout << "Showing octree level: " << octree_show_level << std::endl;
		time_since_last_frame = glfwGetTime();
	}
}


void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
	const float _xpos = (float)xpos;
	const float _ypos = (float)ypos;
	if (firstMouse) {
		lastX = _xpos;
		lastY = _ypos;
		firstMouse = false;
	}

	const float xoffset = _xpos - lastX;
	const float yoffset = lastY - _ypos;
	lastX = _xpos;
	lastY = _ypos;

	camera.ProcessMouseMovement(xoffset, yoffset);

}

// is currently being used to set focus. Could be used for altering focal length
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	camera.ProcessMouseScroll((float)yoffset);
}





#include "sancho.h"

// settings
const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 800;
const float Z_NEAR = 0.1f;
const float Z_FAR = 100.0f;
const float ASPECT_RATIO = ((float)SCR_WIDTH) / SCR_HEIGHT;
static const float SCALE = 1.0f / static_cast<float>(SCR_WIDTH);
static const float PI = 3.14159265359f;

// camera
//Camera camera(-2.0f, 1.5f, 3.4f, 0.0f, 1.0f, 0.0f, -50.0f, -12.0f);
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

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
	//glDepthFunc(GL_LESS);
	fprintf(stderr, "OpenGL %s\n", glGetString(GL_VERSION));

	PointCloud point_cloud;
	load_point_cloud("../test.txt", 4, 3, point_cloud);

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
	float height_of_near_plane = (float)abs(viewport[3] - viewport[1]) /
		(2 * tan(0.5*fovy*PI / 180.0));

	point_cloud_shader.use();
	point_cloud_shader.setVec3("color", 0.9f, 0.4f, 0.0f);
	point_cloud_shader.setFloat("point_size", 0.3f);
	point_cloud_shader.setFloat("z_near", Z_NEAR);
	point_cloud_shader.setFloat("z_far", Z_FAR);
	point_cloud_shader.setFloat("height_of_near_plane", height_of_near_plane);

	Timer t, totalTime;

	while (!glfwWindowShouldClose(window))
	{
		process_input(window);

		const double currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		glClearColor(0.1f, 0.1f, 0.4f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, Z_NEAR, Z_FAR);
		glm::mat4 view = camera.GetViewMatrix();
		glm::mat4 model = glm::mat4(1.0f);
		glm::mat4 MVPmatrix = projection * view * model;

		point_cloud_shader.use();
		point_cloud_shader.setMat4("mvp_matrix", MVPmatrix);
		point_cloud_shader.setVec3("cam_pos", camera.Position);

		glBindVertexArray(VAO);
		glDrawArrays(GL_POINTS, 0, point_cloud.no_of_points);
		
		
		glBindVertexArray(0);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}   

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


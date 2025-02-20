#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>

#include "ifs.h"
#include "engine/engine.h"

struct SpContext *g_ctx;
struct GLFWwindow *g_window;
unsigned int g_windowWidth = 800;
unsigned int g_windowHeight = 600;

void cbckGlfwKey(GLFWwindow *const p_window, int const p_key, int const p_scancode, int const p_action, int const p_mods) {
	if (p_key == GLFW_KEY_ESCAPE) {

		glfwSetWindowShouldClose(g_window, 1);

	}
}

int main(void) {
	ifu(!glfwInit()) {

		perror("GLFW initialization failed!\n");
		exit(EXIT_FAILURE);

	}

	char const windowTitle[] = "Simple Physics Engine Particles Test";
	g_window = glfwCreateWindow(

		g_windowWidth,
		g_windowHeight,
		windowTitle,
		NULL,
		NULL

	);

	ifu(!g_window) {

		perror("GLFW g_window creation failed!\n");
		glfwTerminate();
		exit(EXIT_FAILURE);

	}

	glfwMakeContextCurrent(g_window);
	glfwSetKeyCallback(g_window, cbckGlfwKey);
	glViewport(0, 0, g_windowWidth, g_windowHeight);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, g_windowWidth, 0, g_windowHeight, -1, 1); // "L, R, Bot, Top, Near, Far"!

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	g_ctx = spContextAlloc().result.value;

	const int numParticles = 1000;
	for (int i = 0; i < numParticles; ++i) {

		float const x = rand() % g_windowWidth;
		float const y = g_windowHeight + (rand() % g_windowHeight);
		sp_body_t const body = spBodyCreate(g_ctx).result.value;

		float const weird = 2 * sin(i * x / y);
		spBodySetMass(g_ctx, body, weird > 0 ? weird : -weird);
		spBodySetPosition(g_ctx, body, x, y, 0);

	}

	double const timeStart = glfwGetTime();
	double timeFrameStartPrevious = timeStart;
	size_t frameCount = 1;

	while (!glfwWindowShouldClose(g_window)) {

		double const timeFrameStartCurrent = glfwGetTime();
		float const dt = timeFrameStartCurrent - timeFrameStartPrevious;
		timeFrameStartPrevious = timeFrameStartCurrent;

		if (frameCount % 30 == 0) {

			printf("%lf FPS.\n", 10000 * (timeFrameStartCurrent - timeStart) / frameCount);

		}

		glLoadIdentity();
		glClear(GL_COLOR_BUFFER_BIT);

		for (size_t i = 0; i < g_ctx->ctxTrans->countActive; ++i) {

			sp_body_t const body = g_ctx->ctxTrans->active[i];
			spBodyForceCenter(g_ctx, body, 0, -9.81f, 0);

		}

		spSolveTranslationEuler(g_ctx->ctxTrans, dt);
		glPointSize(15.0f);
		glBegin(GL_POINTS);
		for (size_t i = 0; i < g_ctx->ctxTrans->countActive; ++i) {

			sp_body_t const body = g_ctx->ctxTrans->active[i];

			float const x = spBodyGetPosX(g_ctx, body);
			float const y = spBodyGetPosY(g_ctx, body);
			float const mass = spBodyGetMass(g_ctx, body);

			glColor3f(x / g_windowWidth, y / g_windowHeight, mass);
			glRotatef(15, 0, 0, 1);
			glVertex2f(x, y);

		}
		glEnd();
		glfwPollEvents();
		glfwSwapBuffers(g_window);

		++frameCount;
	}

	glfwDestroyWindow(g_window);
	spContextFree(g_ctx);
	glfwTerminate();

	return 0;
}

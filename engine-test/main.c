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

char g_reset = 0;
int g_windowX = 0;
int g_windowY = 0;
char g_fullScreen = 0;
struct SpContext *g_ctx;
int g_windowWidth = 800;
int g_windowHeight = 600;
struct GLFWwindow *g_window;

void bodiesCreate() {
	for (int i = 0; i < 100000; ++i) {

		float const x = rand() % g_windowWidth;
		float const y = g_windowHeight + (rand() % g_windowHeight);
		sp_body_t const body = spBodyCreate(g_ctx);

		float const weird = 2 * sin(i * x / y);
		spBodySetMass(g_ctx, body, weird > 0 ? weird : -weird);
		spBodySetPosition(g_ctx, body, x, y, 0);

	}
}

void cbckGlfwKey(GLFWwindow *const p_window, int const p_key, int const p_scancode, int const p_action, int const p_mods) {
	ifu(p_key == GLFW_KEY_ESCAPE) {

		glfwSetWindowShouldClose(g_window, 1);

	}

	// `Alt`-`Enter` / `F11` fullscreen:
	ifu(p_action == GLFW_PRESS && (p_key == GLFW_KEY_F11 || p_key == GLFW_KEY_ENTER && (p_mods & GLFW_MOD_ALT))) {

		ifu(g_fullScreen) {

			glfwSetWindowMonitor(g_window, NULL, g_windowX, g_windowY, g_windowWidth, g_windowHeight, GLFW_DONT_CARE);

		} else {

			glfwGetWindowPos(g_window, &g_windowX, &g_windowY);
			glfwGetWindowSize(g_window, &g_windowWidth, &g_windowHeight);

			const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
			glfwSetWindowMonitor(g_window, glfwGetPrimaryMonitor(), 0, 0, mode->width, mode->height, GLFW_DONT_CARE);

		}

		g_fullScreen = !g_fullScreen;

	}

	ifu(p_key == GLFW_KEY_SPACE) {

		g_reset = 1;

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

	// glfwSetWindowSize(g_window, 128, 128);

	glfwSwapInterval(0);
	glfwMakeContextCurrent(g_window);
	glfwSetKeyCallback(g_window, cbckGlfwKey);
	glViewport(0, 0, g_windowWidth, g_windowHeight);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, g_windowWidth, 0, g_windowHeight, -1, 1); // "L, R, Bot, Top, Near, Far"!

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	g_ctx = spContextAlloc();
	bodiesCreate();

	double const timeStart = glfwGetTime();
	double timeFrameStartPrevious = timeStart;
	size_t frameCount = 1;

	while (!glfwWindowShouldClose(g_window)) {

		double const timeFrameStartCurrent = glfwGetTime();
		float const dt = timeFrameStartCurrent - timeFrameStartPrevious;
		timeFrameStartPrevious = timeFrameStartCurrent;

		glfwGetFramebufferSize(g_window, &g_windowWidth, &g_windowHeight);
		glfwGetWindowPos(g_window, &g_windowX, &g_windowY);
		glViewport(0, 0, g_windowWidth, g_windowHeight);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		glOrtho(0, g_windowWidth, 0, g_windowHeight, -1, 1); // "L, R, Bot, Top, Near, Far"!

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

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

		ifu(g_reset) {

			spContextFree(g_ctx);
			g_ctx = spContextAlloc();
			bodiesCreate();
			g_reset = 0;

		}

		++frameCount;
	}

	glfwDestroyWindow(g_window);
	spContextFree(g_ctx);
	glfwTerminate();

	return 0;
}

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <GLFW/glfw3.h>

#include "ifs.h"
#include "engine/engine.h"

#define WINDOW_WIDTH  	800
#define WINDOW_HEIGHT	600

int main(void) {
	ifu(!glfwInit()) {

		perror("GLFW initialization failed!\n");
		exit(EXIT_FAILURE);

	}

	char const windowTitle[] = "Simple Physics Engine Particles Test";
	GLFWwindow *const window = glfwCreateWindow(

		WINDOW_WIDTH,
		WINDOW_HEIGHT,
		windowTitle,
		NULL,
		NULL

	);

	ifu(!window) {

		perror("GLFW window creation failed!\n");
		glfwTerminate();
		exit(EXIT_FAILURE);

	}

	glfwMakeContextCurrent(window);

	glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// Left, Right, Bottom, Top, Near, Far
	glOrtho(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	struct SpContext *ctx = spContextAlloc().result.value;

	srand(time(NULL));

	const int numParticles = 1000;
	for (int i = 0; i < numParticles; ++i) {

		float const x = rand() % WINDOW_WIDTH;
		float const y = WINDOW_HEIGHT + (rand() % WINDOW_HEIGHT);
		sp_body_t const body = spBodyCreate(ctx).result.value;

		spBodySetPosition(ctx, body, x, y, 0);
		spBodySetMass(ctx, body, 2 * sin(i * x / y));
		// spBodySetMass(ctx, body, 100.0f * (rand() / RAND_MAX));

	}

	double const timeStart = glfwGetTime();
	double timeFrameStartPrevious = timeStart;
	unsigned long long frameCount = 0;

	while (!glfwWindowShouldClose(window)) {

		double const timeFrameStartCurrent = glfwGetTime();
		float const dt = timeFrameStartCurrent - timeFrameStartPrevious;
		timeFrameStartPrevious = timeFrameStartCurrent;

		char windowTitleBuf[sizeof(windowTitle) + sizeof(" - (1234567890 FPS)")] = { 0 };
		snprintf(

			windowTitleBuf,
			sizeof(windowTitleBuf),
			"%s - (%lf FPS)",
			windowTitle,
			10000 * (timeFrameStartCurrent - timeStart) / frameCount

		);

		glfwSetWindowTitle(window, windowTitleBuf);

		glLoadIdentity();
		glClear(GL_COLOR_BUFFER_BIT);

		for (unsigned long long i = 0; i < ctx->ctxTrans->countActive; ++i) {

			sp_body_t const body = ctx->ctxTrans->active[i];
			spBodyForceCenter(ctx, body, 0, -9.81f, 0);

		}

		spSolveTranslationEuler(ctx->ctxTrans, dt);
		glPointSize(15.0f);
		glBegin(GL_POINTS);
		for (unsigned long long i = 0; i < ctx->ctxTrans->countActive; ++i) {

			sp_body_t const body = ctx->ctxTrans->active[i];

			float const x = spBodyGetPosX(ctx, body);
			float const y = spBodyGetPosY(ctx, body);
			float const mass = spBodyGetMass(ctx, body);

			glLoadIdentity();
			glRotatef(15, 0, 0, 1);
			glColor3f(x / WINDOW_WIDTH, y / WINDOW_HEIGHT, mass);
			glVertex2f(x, y);

		}
		glEnd();
		glfwPollEvents();
		glfwSwapBuffers(window);
		++frameCount;
	}

	glfwDestroyWindow(window);
	spContextFree(ctx);
	glfwTerminate();

	return 0;
}

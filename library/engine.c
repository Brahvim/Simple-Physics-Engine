#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "ifs.h"
#include "library/sp.h"

sp_size_t g_spBodyDefaultAllocationCount = 16;

#pragma region Solvers.
#pragma region Translation.
void spSolveTranslationEuler(struct SpContextTranslation *const restrict p_ctx, float p_dt) {
	for (sp_size_t i = 0; i < p_ctx->capacityActive; ++i) {

		struct SpVec3 *pos = &p_ctx->data[i].position;
		struct SpVec3 *vel = &p_ctx->data[i].velocity;
		struct SpVec3 *acc = &p_ctx->data[i].acceleration;

		vel->x += acc->x * p_dt;
		vel->y += acc->y * p_dt;
		vel->z += acc->z * p_dt;

		pos->x += vel->x * p_dt;
		pos->y += vel->y * p_dt;
		pos->z += vel->z * p_dt;

		acc->x = acc->y = acc->z = 0;

	}
}

void spSolveTranslationVerlet(struct SpContextTranslation *const restrict p_ctx, float p_dt) {
	for (sp_size_t i = 0; i < p_ctx->capacityActive; ++i) {

		struct SpVec3 *pos = &p_ctx->data[i].position;
		struct SpVec3 *vel = &p_ctx->data[i].velocity;
		struct SpVec3 *acc = &p_ctx->data[i].acceleration;

		struct SpVec3 prev = *pos; // Save current position

		// Update position using Verlet
		pos->x = 2 * pos->x - vel->x + acc->x * p_dt * p_dt;
		pos->y = 2 * pos->y - vel->y + acc->y * p_dt * p_dt;
		pos->z = 2 * pos->z - vel->z + acc->z * p_dt * p_dt;

		// Approximate velocity (if needed for constraints)
		vel->x = (pos->x - prev.x) / p_dt;
		vel->y = (pos->y - prev.y) / p_dt;
		vel->z = (pos->z - prev.z) / p_dt;

		// Reset acceleration
		acc->x = acc->y = acc->z = 0;

	}
}
#pragma endregion

#pragma region Rotation.
#pragma endregion
#pragma endregion

#pragma region `struct SpContext`.
struct SpContext* spContextAlloc() {
	struct SpContext *ctx = malloc(sizeof(struct SpContext));

	ctx->masses = calloc(g_spBodyDefaultAllocationCount, sizeof(float));
	ctx->ctxTrans = spContextTranslationAlloc(); // NOLINT clang-analyzer.unix.Malloc

	ctx->capacityMasses = g_spBodyDefaultAllocationCount;
	ctx->maxId = 0;

	return ctx;
}

void spContextFree(struct SpContext *const restrict p_ctx) {
	spContextTranslationFree(p_ctx->ctxTrans);
	// spContextRotationFree(p_ctx->ctxRot);
	free(p_ctx->masses);
	free(p_ctx);
}
#pragma endregion

#pragma region Bodies!
void spBodyDestroy(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	spContextTranslationDestroyEntry(p_ctx->ctxTrans, p_body);
}

sp_body_t spBodyCreate(struct SpContext *const restrict p_ctx) {
	sp_body_t const id = p_ctx->maxId;

	spContextTranslationCreateEntry(p_ctx->ctxTrans, id);

	ifu(p_ctx->maxId >= p_ctx->capacityMasses) {

		p_ctx->masses = realloc(p_ctx->masses, sizeof(float) * (p_ctx->capacityMasses *= 2));

	}

	p_ctx->maxId++;
	p_ctx->masses[id] = 0;
	return id;
}
#pragma endregion

#pragma region `struct SpContextRotation`.
struct SpContextRotation* spContextRotationAlloc() {
	struct SpContextRotation *ctx = malloc(sizeof(struct SpContextRotation)); // We zero *everything* later...

	ctx->active = calloc(g_spBodyDefaultAllocationCount, sizeof(sp_size_t));

	ctx->capacityActive = g_spBodyDefaultAllocationCount;
	ctx->inactive = calloc(g_spBodyDefaultAllocationCount, sizeof(sp_size_t));

	ctx->capacityInactive = g_spBodyDefaultAllocationCount;
	ctx->data = calloc(1 + g_spBodyDefaultAllocationCount, sizeof(struct SpSolverParametersRotation));

	// Add new handles to `ctx::inactive`:
	// for (sp_size_t i = g_spBodyDefaultAllocationCount - 1; i > 0; --i) {
	for (sp_size_t i = 0; i < g_spBodyDefaultAllocationCount; ++i) {

		ctx->inactive[i] = i;

	}

	ctx->countInactive = g_spBodyDefaultAllocationCount;
	ctx->countActive = 0;

	return ctx;
}

void spContextRotationFree(struct SpContextRotation *const restrict p_ctx) {
	free(p_ctx->inactive);
	free(p_ctx->active);
	free(p_ctx->data);
	free(p_ctx);
}

void spContextRotationCreateEntry(struct SpContextRotation *const restrict p_ctx, sp_body_t const p_body) {
	ifl(p_ctx->countInactive > 0) { // Grab body from free-list.

		p_ctx->countInactive--;
		p_body = p_ctx->inactive[p_ctx->countInactive];

	} else ifu(p_ctx->countActive >= p_ctx->capacityActive) {

		p_ctx->active = realloc(p_ctx->active, 2 * sizeof(sp_size_t) * p_ctx->capacityActive);
		p_ctx->capacityActive *= 2;
		p_ctx->data = realloc(p_ctx->data, 2 * sizeof(struct SpSolverParametersRotation) * (1 + p_ctx->capacityActive + p_ctx->capacityInactive));

	}

	memset(&p_ctx->data[p_body], 0, sizeof(struct SpSolverParametersRotation)); // NOLINT clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling
	p_ctx->active[p_ctx->countActive] = p_body;
	p_ctx->countActive++;
}

void spContextRotationDestroyEntry(struct SpContextRotation *const restrict p_ctx, sp_body_t const p_body) {
	ifu(p_ctx->countInactive >= p_ctx->capacityInactive) {

		p_ctx->inactive = realloc(p_ctx->inactive, sizeof(sp_size_t) * (p_ctx->capacityInactive *= 2));

	}

	for (sp_size_t i = 0; i < p_ctx->countActive; ++i) {

		ifu(p_ctx->active[i] == p_body) {

			p_ctx->active[i] = p_ctx->active[p_ctx->countActive - 1];
			p_ctx->countActive--;
			p_ctx->inactive[p_ctx->countInactive] = p_body;
			p_ctx->countInactive++;

			return;

		}

	}
}
#pragma endregion

#pragma region `struct SpContextTranslation`.
struct SpContextTranslation* spContextTranslationAlloc() {
	struct SpContextTranslation *ctx = malloc(sizeof(struct SpContextTranslation)); // We zero *everything* later...

	ctx->active = calloc(g_spBodyDefaultAllocationCount, sizeof(sp_size_t));

	ctx->capacityActive = g_spBodyDefaultAllocationCount;
	ctx->inactive = calloc(g_spBodyDefaultAllocationCount, sizeof(sp_size_t));

	ctx->capacityInactive = g_spBodyDefaultAllocationCount;
	ctx->data = calloc(1 + g_spBodyDefaultAllocationCount, sizeof(struct SpSolverParametersTranslation));

	// Add new handles to `ctx::inactive`:
	// for (sp_size_t i = g_spBodyDefaultAllocationCount - 1; i > 0; --i) {
	for (sp_size_t i = 0; i < g_spBodyDefaultAllocationCount; ++i) {

		ctx->inactive[i] = i;

	}

	ctx->countInactive = g_spBodyDefaultAllocationCount;
	ctx->countActive = 0;

	return ctx;
}

void spContextTranslationFree(struct SpContextTranslation *const restrict p_ctx) {
	free(p_ctx->active);
	free(p_ctx->inactive);
	free(p_ctx->data);
	free(p_ctx);
}

void spContextTranslationCreateEntry(struct SpContextTranslation *const restrict p_ctx, sp_body_t const p_body) {
	ifl(p_ctx->countInactive > 0) { // Grab body from free-list.

		p_ctx->countInactive--;
		p_body = p_ctx->inactive[p_ctx->countInactive];

	} else ifu(p_ctx->countActive >= p_ctx->capacityActive) {

		p_ctx->active = realloc(p_ctx->active, 2 * sizeof(sp_size_t) * p_ctx->capacityActive);
		p_ctx->capacityActive *= 2;
		p_ctx->data = realloc(p_ctx->data, 2 * sizeof(struct SpSolverParametersTranslation) * (1 + p_ctx->capacityActive + p_ctx->capacityInactive));

	}

	memset(&p_ctx->data[p_body], 0, sizeof(struct SpSolverParametersTranslation)); // NOLINT clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling
	p_ctx->active[p_ctx->countActive] = p_body;
	p_ctx->countActive++;
}

void spContextTranslationDestroyEntry(struct SpContextTranslation *const restrict p_ctx, sp_body_t const p_body) {
	ifu(p_ctx->countInactive >= p_ctx->capacityInactive) {

		p_ctx->inactive = realloc(p_ctx->inactive, sizeof(sp_size_t) * (p_ctx->capacityInactive *= 2));

	}

	for (sp_size_t i = 0; i < p_ctx->countActive; ++i) {

		ifu(p_ctx->active[i] == p_body) {

			p_ctx->active[i] = p_ctx->active[p_ctx->countActive - 1];
			p_ctx->countActive--;
			p_ctx->inactive[p_ctx->countInactive] = p_body;
			p_ctx->countInactive++;

			return;

		}

	}
}
#pragma endregion

#pragma region Millions of getters, setters, and modifiers.
#pragma region Getters.
float spBodyGetMass(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->masses[p_body];
}

float spBodyGetPosX(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxTrans->data[p_body].position.x;
}

float spBodyGetPosY(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxTrans->data[p_body].position.y;
}

float spBodyGetPosZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxTrans->data[p_body].position.z;
}

struct SpVec3* spBodyGetPosition(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return &p_ctx->ctxTrans->data[p_body].position;
}

float spBodyGetVelX(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxTrans->data[p_body].velocity.x;
}

float spBodyGetVelY(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxTrans->data[p_body].velocity.y;
}

float spBodyGetVelZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxTrans->data[p_body].velocity.z;
}

struct SpVec3* spBodyGetVelocity(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return &p_ctx->ctxTrans->data[p_body].velocity;
}

float spBodyGetAccX(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxTrans->data[p_body].acceleration.x;
}

float spBodyGetAccY(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxTrans->data[p_body].acceleration.y;
}

float spBodyGetAccZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxTrans->data[p_body].acceleration.z;
}

struct SpVec3* spBodyGetAcceleration(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return &p_ctx->ctxTrans->data[p_body].acceleration;
}

float spBodyGetAngX(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxRot->data[p_body].angles.z;
}

float spBodyGetAngY(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxRot->data[p_body].angles.y;
}

float spBodyGetAngZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxRot->data[p_body].angles.z;
}

struct SpVec3* spBodyGetAngles(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return &p_ctx->ctxRot->data[p_body].angles;
}

float spBodyGetVelAngX(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxRot->data[p_body].angularVelocity.x;
}

float spBodyGetVelAngY(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxRot->data[p_body].angularVelocity.y;
}

float spBodyGetVelAngZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxRot->data[p_body].angularVelocity.z;
}

struct SpVec3* spBodyGetAngularVelocity(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return &p_ctx->ctxRot->data[p_body].angularVelocity;
}

float spBodyGetAccAngX(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxRot->data[p_body].angularAcceleration.x;
}

float spBodyGetAccAngY(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxRot->data[p_body].angularAcceleration.y;
}

float spBodyGetAccAngZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return p_ctx->ctxRot->data[p_body].angularAcceleration.z;
}

struct SpVec3* spBodyGetAngularAcceleration(struct SpContext *const restrict p_ctx, sp_body_t const p_body) {
	return &p_ctx->ctxRot->data[p_body].angularAcceleration;
}
#pragma endregion

#pragma region Setters.
void spBodySetMass(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->masses[p_body] = p_value;
}

void spBodySetPosX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].position.x = p_value;
}

void spBodySetPosY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].position.y = p_value;
}

void spBodySetPosZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].position.z = p_value;
}

void spBodySetPosition(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxTrans->data[p_body].position.x = p_x;
	p_ctx->ctxTrans->data[p_body].position.y = p_y;
	p_ctx->ctxTrans->data[p_body].position.z = p_z;
}

void spBodySetVelX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].velocity.x = p_value;
}

void spBodySetVelY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].velocity.y = p_value;
}

void spBodySetVelZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].velocity.z = p_value;
}

void spBodySetVelocity(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxTrans->data[p_body].velocity.x = p_x;
	p_ctx->ctxTrans->data[p_body].velocity.y = p_y;
	p_ctx->ctxTrans->data[p_body].velocity.z = p_z;
}

void spBodySetAccX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].acceleration.x = p_value;
}

void spBodySetAccY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].acceleration.y = p_value;
}

void spBodySetAccZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].acceleration.z = p_value;
}

void spBodySetAcceleration(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxTrans->data[p_body].acceleration.x = p_x;
	p_ctx->ctxTrans->data[p_body].acceleration.y = p_y;
	p_ctx->ctxTrans->data[p_body].acceleration.z = p_z;
}

void spBodySetAngX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angles.z = p_value;
}

void spBodySetAngY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angles.y = p_value;
}

void spBodySetAngZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angles.z = p_value;
}

void spBodySetAngles(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxRot->data[p_body].angles.x = p_x;
	p_ctx->ctxRot->data[p_body].angles.y = p_y;
	p_ctx->ctxRot->data[p_body].angles.z = p_z;
}

void spBodySetVelAngX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularVelocity.x = p_value;
}

void spBodySetVelAngY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularVelocity.y = p_value;
}

void spBodySetVelAngZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularVelocity.z = p_value;
}

void spBodySetVelocityAngular(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxRot->data[p_body].angularVelocity.x = p_x;
	p_ctx->ctxRot->data[p_body].angularVelocity.y = p_y;
	p_ctx->ctxRot->data[p_body].angularVelocity.z = p_z;
}

void spBodySetAccAngX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularAcceleration.x = p_value;
}

void spBodySetAccAngY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularAcceleration.y = p_value;
}

void spBodySetAccAngZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularAcceleration.z = p_value;
}

void spBodySetAccelerationAngular(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxRot->data[p_body].angularAcceleration.x = p_x;
	p_ctx->ctxRot->data[p_body].angularAcceleration.y = p_y;
	p_ctx->ctxRot->data[p_body].angularAcceleration.z = p_z;
}
#pragma endregion

#pragma region Modifiers.
void spBodyAddMass(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->masses[p_body] += p_value;
}

void spBodyAddPosX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].position.x += p_value;
}

void spBodyAddPosY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].position.y += p_value;
}

void spBodyAddPosZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].position.z += p_value;
}

void spBodyAddPosition(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxTrans->data[p_body].position.x += p_x;
	p_ctx->ctxTrans->data[p_body].position.y += p_y;
	p_ctx->ctxTrans->data[p_body].position.z += p_z;
}

void spBodyAddVelX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].velocity.x += p_value;
}

void spBodyAddVelY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].velocity.y += p_value;
}

void spBodyAddVelZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].velocity.z += p_value;
}

void spBodyAddVelocity(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxTrans->data[p_body].velocity.x += p_x;
	p_ctx->ctxTrans->data[p_body].velocity.y += p_y;
	p_ctx->ctxTrans->data[p_body].velocity.z += p_z;
}

void spBodyAddAccX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].acceleration.x += p_value;
}

void spBodyAddAccY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].acceleration.y += p_value;
}

void spBodyAddAccZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxTrans->data[p_body].acceleration.z += p_value;
}

void spBodyAddAcceleration(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxTrans->data[p_body].acceleration.x += p_x;
	p_ctx->ctxTrans->data[p_body].acceleration.y += p_y;
	p_ctx->ctxTrans->data[p_body].acceleration.z += p_z;
}

void spBodyAddAngX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angles.z += p_value;
}

void spBodyAddAngY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angles.y += p_value;
}

void spBodyAddAngZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angles.z += p_value;
}

void spBodyAddAngles(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxRot->data[p_body].angles.x += p_x;
	p_ctx->ctxRot->data[p_body].angles.y += p_y;
	p_ctx->ctxRot->data[p_body].angles.z += p_z;
}

void spBodyAddVelAngX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularVelocity.z += p_value;
}

void spBodyAddVelAngY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularVelocity.y += p_value;
}

void spBodyAddVelAngZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularVelocity.z += p_value;
}

void spBodyAddVelocityAngular(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxRot->data[p_body].angularVelocity.x += p_x;
	p_ctx->ctxRot->data[p_body].angularVelocity.y += p_y;
	p_ctx->ctxRot->data[p_body].angularVelocity.z += p_z;
}

void spBodyAddAccAngX(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularAcceleration.z += p_value;
}

void spBodyAddAccAngY(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularAcceleration.y += p_value;
}

void spBodyAddAccAngZ(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_value) {
	p_ctx->ctxRot->data[p_body].angularAcceleration.z += p_value;
}

void spBodyAddAccelerationAngular(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_x, float p_y, float p_z) {
	p_ctx->ctxRot->data[p_body].angularAcceleration.x += p_x;
	p_ctx->ctxRot->data[p_body].angularAcceleration.y += p_y;
	p_ctx->ctxRot->data[p_body].angularAcceleration.z += p_z;
}

void spBodyForceCenter(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_fx, float p_fy, float p_fz) {
	float const mass = spBodyGetMass(p_ctx, p_body);
	p_ctx->ctxTrans->data[p_body].position.x += p_fx / mass;
	p_ctx->ctxTrans->data[p_body].position.y += p_fy / mass;
	p_ctx->ctxTrans->data[p_body].position.z += p_fz / mass;
}

void spBodyForce(struct SpContext *const restrict p_ctx, sp_body_t const p_body, float p_fx, float p_fy, float p_fz, float p_px, float p_py, float p_pz) {
	float const mass = spBodyGetMass(p_ctx, p_body);

	float const diffX = spBodyGetPosX(p_ctx, p_body) - p_px;
	float const diffY = spBodyGetPosY(p_ctx, p_body) - p_py;
	float const diffZ = spBodyGetPosZ(p_ctx, p_body) - p_pz;

	// Here lies a cross-product:
	float const torqueX = p_fy * diffZ - diffY * p_fz;
	float const torqueY = p_fz * diffX - diffZ * p_fx;
	float const torqueZ = p_fx * diffY - diffX * p_fy;

	// Replace *mass* with *moment of inertia! SOMEDAY!:*
	p_ctx->ctxRot->data[p_body].angularAcceleration.x += torqueX / mass;
	p_ctx->ctxRot->data[p_body].angularAcceleration.y += torqueY / mass;
	p_ctx->ctxRot->data[p_body].angularAcceleration.z += torqueZ / mass;
}

#pragma endregion
#pragma endregion

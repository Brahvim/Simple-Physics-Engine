#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "engine/engine.h"

unsigned long long g_spBodyDefaultAllocationCount = 16;

#pragma region Solvers.
void spSolveTranslationEuler(struct SpManagerBodyTranslation *p_man, float p_dt) {
	for (unsigned long long i = 0; i < 3 * p_man->capacityActive; i += 3) {

		struct SpVec3 *pos = p_man->data + i;
		struct SpVec3 *vel = pos + 1;
		struct SpVec3 *acc = vel + 1;

		vel->x += acc->x * p_dt;
		vel->y += acc->y * p_dt;
		vel->z += acc->z * p_dt;

		pos->x += vel->x * p_dt;
		pos->y += vel->y * p_dt;
		pos->z += vel->z * p_dt;

		acc->x = 0;
		acc->y = 0;
		acc->z = 0;

	}
}

void spSolveTranslationVerlet(struct SpManagerBodyTranslation *p_man, float p_dt) {
	float dt2 = p_dt * p_dt; // Squared delta time

	for (unsigned long long i = 0; i < 3 * p_man->capacityActive; i += 3) {
		struct SpVec3 *pos = p_man->data + i;
		struct SpVec3 *vel = pos + 1;
		struct SpVec3 *acc = vel + 1;

		struct SpVec3 prev_pos = *pos; // Save current position

		// Update position using Verlet
		pos->x = 2 * pos->x - vel->x + acc->x * dt2;
		pos->y = 2 * pos->y - vel->y + acc->y * dt2;
		pos->z = 2 * pos->z - vel->z + acc->z * dt2;

		// Approximate velocity (if needed for constraints)
		vel->x = (pos->x - prev_pos.x) / p_dt;
		vel->y = (pos->y - prev_pos.y) / p_dt;
		vel->z = (pos->z - prev_pos.z) / p_dt;

		// Reset acceleration
		acc->x = acc->y = acc->z = 0;
	}
}

#pragma endregion

#pragma region `struct SpContextBody`.
struct SpResultPointer spContextBodyAlloc() {
	struct SpContext *ctx = malloc(sizeof(struct SpContext));

	if (!ctx) {

		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	ctx->masses = calloc(g_spBodyDefaultAllocationCount, sizeof(float));
	ctx->manTrans = spManagerBodyTranslationAlloc().result.value; // NOLINT clang-analyzer.unix.Malloc
	ctx->capacityMasses = g_spBodyDefaultAllocationCount;
	ctx->maxId = 0;

	return (struct SpResultPointer) { .bad = 0, .result.value = ctx }; // cppcheck-suppress memleak
}

sp_error_t spContextBodyFree(struct SpContext *p_ctx) {
	if (!p_ctx) {

		return PHYSICS_ERROR_OBJECT_NULL;

	}

	spManagerBodyTranslationFree(p_ctx->manTrans);
	free(p_ctx->masses);
	free(p_ctx);

	return PHYSICS_ERROR_NONE;
}
#pragma endregion

#pragma region Bodies!
struct SpResultIntegerUnsigned spBodyCreate(struct SpContext *p_ctx) {
	sp_body_t const id = p_ctx->maxId;

	spManagerBodyTranslationCreateEntry(p_ctx->manTrans, id);

	if (p_ctx->maxId >= p_ctx->capacityMasses) {

		if (p_ctx->capacityMasses < 1) {

			p_ctx->capacityMasses = g_spBodyDefaultAllocationCount;

		}

		unsigned long long cap = 2 * p_ctx->capacityMasses;
		float *masses = realloc(p_ctx->masses, cap * sizeof(float));

		if (!masses) {

			return (struct SpResultIntegerUnsigned) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

		}

		// "Doin' what `realloc()` don't!":
		// memset(masses + p_ctx->capacityMasses, 0, sizeof(float) * (cap - p_ctx->capacityMasses));

		p_ctx->masses = masses;
		p_ctx->capacityMasses = cap;

	}

	p_ctx->maxId++;
	p_ctx->masses[id] = 0;
	return (struct SpResultIntegerUnsigned) { .bad = 0, .result.value = id };
}

sp_error_t spBodyDestroy(struct SpContext *p_ctx, sp_body_t p_body) {
	spManagerBodyTranslationDestroyEntry(p_ctx->manTrans, p_body);
	return PHYSICS_ERROR_NONE;
}
#pragma endregion

#pragma region `struct SpManagerBodyRotation`.
struct SpResultPointer spManagerBodyRotationAlloc() {
	struct SpManagerBodyRotation *man = malloc(sizeof(struct SpManagerBodyRotation)); // We zero *everything* later...
	if (!man) {

		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	man->active = calloc(g_spBodyDefaultAllocationCount, sizeof(unsigned long long));
	if (!man->active) {

		free(man);
		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	man->capacityActive = g_spBodyDefaultAllocationCount;
	man->inactive = calloc(g_spBodyDefaultAllocationCount, sizeof(unsigned long long));
	if (!man->inactive) {

		free(man->active);
		free(man);
		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	man->capacityInactive = g_spBodyDefaultAllocationCount;
	man->data = calloc(3 * (1 + g_spBodyDefaultAllocationCount), sizeof(struct SpVec3));
	if (!man->data) {

		free(man->active);
		free(man->inactive);
		free(man);
		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	// Add new handles to `man::inactive`:
	// for (unsigned long long i = g_spBodyDefaultAllocationCount - 1; i > 0; --i) {
	for (unsigned long long i = 0; i < g_spBodyDefaultAllocationCount; ++i) {

		man->inactive[i] = i;

	}

	man->countInactive = g_spBodyDefaultAllocationCount;
	man->countActive = 0;

	return (struct SpResultPointer) { .bad = 0, .result.value = man };
}

sp_error_t spManagerBodyRotationFree(struct SpManagerBodyRotation *p_man) {
	if (!p_man) {

		return PHYSICS_ERROR_OBJECT_NULL;

	}

	free(p_man->inactive);
	free(p_man->active);
	free(p_man->data);
	free(p_man);

	return PHYSICS_ERROR_NONE;
}

sp_error_t spManagerBodyRotationCreateEntry(struct SpManagerBodyRotation *p_man, sp_body_t p_body) {
	if (p_man->countInactive > 0) { // Grab body from free-list.

		p_man->countInactive--;
		p_body = p_man->inactive[p_man->countInactive];

	}

	if (p_man->countActive >= p_man->capacityActive) {

		if (p_man->capacityActive < 1) {

			p_man->capacityActive = g_spBodyDefaultAllocationCount;

		}

		void *active = realloc(p_man->active, 2 * sizeof(unsigned long long) * p_man->capacityActive);

		if (!active) {

			// spManagerBodyRotationFree(p_man); // Yep! Free the *whole* manager.
			return PHYSICS_ERROR_OUT_OF_MEMORY;

		}

		p_man->active = active;
		p_man->capacityActive *= 2;

		void *data = realloc(p_man->data, 2 * 3 * sizeof(struct SpVec3) * (1 + p_man->capacityActive + p_man->capacityInactive));

		if (!data) {

			// spManagerBodyRotationFree(p_man); // Yep! Free the *whole* manager.
			return PHYSICS_ERROR_OUT_OF_MEMORY;

		}

		p_man->data = data;

	}

	memset(&p_man->data[p_body], 0, 3 * sizeof(struct SpVec3)); // NOLINT clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling
	p_man->active[p_man->countActive] = p_body;
	p_man->countActive++;

	return PHYSICS_ERROR_NONE;
}

sp_error_t spManagerBodyRotationDestroyEntry(struct SpManagerBodyRotation *p_man, sp_body_t p_body) {
	if (p_man->countInactive >= p_man->capacityInactive) {

		if (p_man->capacityInactive < 1) {

			p_man->capacityInactive = g_spBodyDefaultAllocationCount;

		}

		void *inactive = realloc(p_man->inactive, 2 * sizeof(unsigned long long) * p_man->capacityInactive);

		if (!inactive) {

			// spManagerBodyRotationFree(p_man); // Yep! Free the *whole* manager.
			return PHYSICS_ERROR_OUT_OF_MEMORY;

		}

		p_man->inactive = inactive;
		p_man->capacityInactive *= 2;

	}

	for (unsigned long long i = 0; i < p_man->countActive; ++i) {

		if (p_man->active[i] == p_body) {

			p_man->active[i] = p_man->active[p_man->countActive - 1];
			p_man->countActive--;
			p_man->inactive[p_man->countInactive] = p_body;
			p_man->countInactive++;

			return PHYSICS_ERROR_NONE;

		}

	}

	return PHYSICS_ERROR_OBJECT_ABSENT;
}
#pragma endregion

#pragma region `struct SpManagerBodyTranslation`.
struct SpResultPointer spManagerBodyTranslationAlloc() {
	struct SpManagerBodyTranslation *man = malloc(sizeof(struct SpManagerBodyTranslation)); // We zero *everything* later...
	if (!man) {

		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	man->active = calloc(g_spBodyDefaultAllocationCount, sizeof(unsigned long long));
	if (!man->active) {

		free(man);
		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	man->capacityActive = g_spBodyDefaultAllocationCount;
	man->inactive = calloc(g_spBodyDefaultAllocationCount, sizeof(unsigned long long));
	if (!man->inactive) {

		free(man->active);
		free(man);
		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	man->capacityInactive = g_spBodyDefaultAllocationCount;
	man->data = calloc(3 * (1 + g_spBodyDefaultAllocationCount), sizeof(struct SpVec3));
	if (!man->data) {

		free(man->active);
		free(man->inactive);
		free(man);
		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	// Add new handles to `man::inactive`:
	// for (unsigned long long i = g_spBodyDefaultAllocationCount - 1; i > 0; --i) {
	for (unsigned long long i = 0; i < g_spBodyDefaultAllocationCount; ++i) {

		man->inactive[i] = i;

	}

	man->countInactive = g_spBodyDefaultAllocationCount;
	man->countActive = 0;

	return (struct SpResultPointer) { .bad = 0, .result.value = man };
}

sp_error_t spManagerBodyTranslationFree(struct SpManagerBodyTranslation *p_man) {
	if (!p_man) {

		return PHYSICS_ERROR_OBJECT_NULL;

	}

	free(p_man->active);
	free(p_man->inactive);
	free(p_man->data);
	free(p_man);

	return PHYSICS_ERROR_NONE;
}

sp_error_t spManagerBodyTranslationCreateEntry(struct SpManagerBodyTranslation *p_man, sp_body_t p_body) {
	if (p_man->countInactive > 0) { // Grab body from free-list.

		p_man->countInactive--;
		p_body = p_man->inactive[p_man->countInactive];

	}

	if (p_man->countActive >= p_man->capacityActive) {

		if (p_man->capacityActive < 1) {

			p_man->capacityActive = g_spBodyDefaultAllocationCount;

		}

		void *active = realloc(p_man->active, 2 * sizeof(unsigned long long) * p_man->capacityActive);

		if (!active) {

			// spManagerBodyTranslationFree(p_man); // Yep! Free the *whole* manager.
			return PHYSICS_ERROR_OUT_OF_MEMORY;

		}

		p_man->active = active;
		p_man->capacityActive *= 2;

		void *data = realloc(p_man->data, 2 * 3 * sizeof(struct SpVec3) * (1 + p_man->capacityActive + p_man->capacityInactive));

		if (!data) {

			// spManagerBodyTranslationFree(p_man); // Yep! Free the *whole* manager.
			return PHYSICS_ERROR_OUT_OF_MEMORY;

		}

		p_man->data = data;

	}

	memset(&p_man->data[p_body], 0, 3 * sizeof(struct SpVec3)); // NOLINT clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling
	p_man->active[p_man->countActive] = p_body;
	p_man->countActive++;

	return PHYSICS_ERROR_NONE;
}

sp_error_t spManagerBodyTranslationDestroyEntry(struct SpManagerBodyTranslation *p_man, sp_body_t p_body) {
	if (p_man->countInactive >= p_man->capacityInactive) {

		if (p_man->capacityInactive < 1) {

			p_man->capacityInactive = g_spBodyDefaultAllocationCount;

		}

		void *inactive = realloc(p_man->inactive, 2 * sizeof(unsigned long long) * p_man->capacityInactive);

		if (!inactive) {

			// spManagerBodyTranslationFree(p_man); // Yep! Free the *whole* manager.
			return PHYSICS_ERROR_OUT_OF_MEMORY;

		}

		p_man->inactive = inactive;
		p_man->capacityInactive *= 2;

	}

	for (unsigned long long i = 0; i < p_man->countActive; ++i) {

		if (p_man->active[i] == p_body) {

			p_man->active[i] = p_man->active[p_man->countActive - 1];
			p_man->countActive--;
			p_man->inactive[p_man->countInactive] = p_body;
			p_man->countInactive++;

			return PHYSICS_ERROR_NONE;

		}

	}

	return PHYSICS_ERROR_OBJECT_ABSENT;
}
#pragma endregion

#pragma region Millions of getters, setters, and modifiers.
#pragma region Getters.
float spBodyGetMass(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->masses[3 * p_body];
}

float spBodyGetPosX(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manTrans->data[3 * p_body].x;
}

float spBodyGetPosY(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manTrans->data[3 * p_body].y;
}

float spBodyGetPosZ(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manTrans->data[3 * p_body].z;
}

struct SpVec3* spBodyGetPos(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return &p_ctx->manTrans->data[3 * p_body];
}

float spBodyGetVelX(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manTrans->data[3 * p_body + 1].x;
}

float spBodyGetVelY(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manTrans->data[3 * p_body + 1].y;
}

float spBodyGetVelZ(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manTrans->data[3 * p_body + 1].z;
}

struct SpVec3* spBodyGetVel(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return &p_ctx->manTrans->data[3 * p_body + 1];
}

float spBodyGetAccX(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manTrans->data[3 * p_body + 2].x;
}

float spBodyGetAccY(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manTrans->data[3 * p_body + 2].y;
}

float spBodyGetAccZ(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manTrans->data[3 * p_body + 2].z;
}

struct SpVec3* spBodyGetAcc(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return &p_ctx->manTrans->data[3 * p_body + 2];
}

float spBodyGetAngX(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manRot->data[3 * p_body].z;
}

float spBodyGetAngY(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manRot->data[3 * p_body].y;
}

float spBodyGetAngZ(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manRot->data[3 * p_body].z;
}

float spBodyGetVelAngX(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manRot->data[3 * p_body + 1].x;
}

float spBodyGetVelAngY(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manRot->data[3 * p_body + 1].y;
}

float spBodyGetVelAngZ(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manRot->data[3 * p_body + 1].z;
}

float spBodyGetAccAngX(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manRot->data[3 * p_body + 2].x;
}

float spBodyGetAccAngY(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manRot->data[3 * p_body + 2].y;
}

float spBodyGetAccAngZ(struct SpContext *restrict p_ctx, sp_body_t p_body) {
	return p_ctx->manRot->data[3 * p_body + 2].z;
}
#pragma endregion

#pragma region Setters.
void spBodySetMass(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->masses[3 * p_body] = p_value;
}

void spBodySetPosX(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body].x = p_value;
}

void spBodySetPosY(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body].y = p_value;
}

void spBodySetPosZ(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body].z = p_value;
}

void spBodySetPosition(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_x, float p_y, float p_z) {
	p_ctx->manTrans->data[3 * p_body].x = p_x;
	p_ctx->manTrans->data[3 * p_body].y = p_y;
	p_ctx->manTrans->data[3 * p_body].z = p_z;
}

void spBodySetVelX(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 1].x = p_value;
}

void spBodySetVelY(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 1].y = p_value;
}

void spBodySetVelZ(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 1].z = p_value;
}

void spBodySetVelocity(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_x, float p_y, float p_z) {
	p_ctx->manTrans->data[3 * p_body + 1].x = p_x;
	p_ctx->manTrans->data[3 * p_body + 1].y = p_y;
	p_ctx->manTrans->data[3 * p_body + 1].z = p_z;
}

void spBodySetAccX(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 2].x = p_value;
}

void spBodySetAccY(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 2].y = p_value;
}

void spBodySetAccZ(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 2].z = p_value;
}

void spBodySetAcceleration(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_x, float p_y, float p_z) {
	p_ctx->manTrans->data[3 * p_body + 2].x = p_x;
	p_ctx->manTrans->data[3 * p_body + 2].y = p_y;
	p_ctx->manTrans->data[3 * p_body + 2].z = p_z;
}

void spBodySetAngX(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manRot->data[3 * p_body].z = p_value;
}

void spBodySetAngY(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manRot->data[3 * p_body].y = p_value;
}

void spBodySetAngZ(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manRot->data[3 * p_body].z = p_value;
}

void spBodySetAngles(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_x, float p_y, float p_z) {
	p_ctx->manRot->data[3 * p_body].x = p_x;
	p_ctx->manRot->data[3 * p_body].y = p_y;
	p_ctx->manRot->data[3 * p_body].z = p_z;
}

void spBodySetVelAngX(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manRot->data[3 * p_body + 1].x = p_value;
}

void spBodySetVelAngY(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manRot->data[3 * p_body + 1].y = p_value;
}

void spBodySetVelAngZ(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manRot->data[3 * p_body + 1].z = p_value;
}

void spBodySetVelocityAngular(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_x, float p_y, float p_z) {
	p_ctx->manRot->data[3 * p_body + 1].x = p_x;
	p_ctx->manRot->data[3 * p_body + 1].y = p_y;
	p_ctx->manRot->data[3 * p_body + 1].z = p_z;
}

void spBodySetAccAngX(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manRot->data[3 * p_body + 2].x = p_value;
}

void spBodySetAccAngY(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manRot->data[3 * p_body + 2].y = p_value;
}

void spBodySetAccAngZ(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manRot->data[3 * p_body + 2].z = p_value;
}

void spBodySetAccelerationAngular(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_x, float p_y, float p_z) {
	p_ctx->manRot->data[3 * p_body + 2].x = p_x;
	p_ctx->manRot->data[3 * p_body + 2].y = p_y;
	p_ctx->manRot->data[3 * p_body + 2].z = p_z;
}
#pragma endregion

#pragma region Modifiers.
void spBodyAddMass(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->masses[3 * p_body] += p_value;
}

void spBodyAddPosX(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body].x += p_value;
}

void spBodyAddPosY(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body].y += p_value;
}

void spBodyAddPosZ(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body].z += p_value;
}

void spBodyAddPosition(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_x, float p_y, float p_z) {
	p_ctx->manTrans->data[3 * p_body].x += p_x;
	p_ctx->manTrans->data[3 * p_body].y += p_y;
	p_ctx->manTrans->data[3 * p_body].z += p_z;
}

void spBodyAddVelX(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 1].x += p_value;
}

void spBodyAddVelY(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 1].y += p_value;
}

void spBodyAddVelZ(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 1].z += p_value;
}

void spBodyAddVelocity(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_x, float p_y, float p_z) {
	p_ctx->manTrans->data[3 * p_body + 1].x += p_x;
	p_ctx->manTrans->data[3 * p_body + 1].y += p_y;
	p_ctx->manTrans->data[3 * p_body + 1].z += p_z;
}

void spBodyAddAccX(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 2].x += p_value;
}

void spBodyAddAccY(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 2].y += p_value;
}

void spBodyAddAccZ(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manTrans->data[3 * p_body + 2].z += p_value;
}

void spBodyAddAcceleration(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_x, float p_y, float p_z) {
	p_ctx->manTrans->data[3 * p_body + 2].x += p_x;
	p_ctx->manTrans->data[3 * p_body + 2].y += p_y;
	p_ctx->manTrans->data[3 * p_body + 2].z += p_z;
}

void spBodyAddAngX(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manRot->data[3 * p_body].z += p_value;
}

void spBodyAddAngY(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_value) {
	p_ctx->manRot->data[3 * p_body].y += p_value;
}


void spBodyForceCenter(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_fx, float p_fy, float p_fz) {
	float const mass = spBodyGetMass(p_ctx, p_body);
	p_ctx->manTrans->data[3 * p_body].x += p_fx / mass;
	p_ctx->manTrans->data[3 * p_body].y += p_fy / mass;
	p_ctx->manTrans->data[3 * p_body].z += p_fz / mass;
}

void spBodyForce(struct SpContext *restrict p_ctx, sp_body_t p_body, float p_fx, float p_fy, float p_fz, float p_px, float p_py, float p_pz) {
	float const mass = spBodyGetMass(p_ctx, p_body);

	float const diffX = spBodyGetPosX(p_ctx, p_body) - p_px;
	float const diffY = spBodyGetPosY(p_ctx, p_body) - p_py;
	float const diffZ = spBodyGetPosZ(p_ctx, p_body) - p_pz;

	// Here lies a cross-product:
	float const torqueX = p_fy * diffZ - diffY * p_fz;
	float const torqueY = p_fz * diffX - diffZ * p_fx;
	float const torqueZ = p_fx * diffY - diffX * p_fy;

	// Replace *mass* with *moment of inertia!:*
	p_ctx->manTrans->data[p_body * 3].x += torqueX / mass;
	p_ctx->manTrans->data[p_body * 3 + 1].y += torqueY / mass;
	p_ctx->manTrans->data[p_body * 3 + 2].z += torqueZ / mass;
}

#pragma endregion
#pragma endregion

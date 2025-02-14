#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "engine/engine.h"

unsigned long long g_spBodyDefaultAllocationCount = 16;

#pragma region Solvers.
void spSolverTranslationEuler(struct SpManagerBodyTranslation *p_man, float p_dt) {
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

void spSolverTranslationVerlet(struct SpManagerBodyTranslation *p_man, float p_dt) {
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
	struct SpContextBody *ctx = malloc(sizeof(struct SpContextBody));

	if (!ctx) {

		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	ctx->masses = calloc(g_spBodyDefaultAllocationCount, sizeof(float));
	ctx->manTrans = spManagerBodyTranslationAlloc().result.value; // NOLINT clang-analyzer.unix.Malloc
	ctx->capacityMasses = g_spBodyDefaultAllocationCount;
	ctx->maxId = 0;

	return (struct SpResultPointer) { .bad = 0, .result.value = ctx }; // cppcheck-suppress memleak
}

sp_error_t spContextBodyFree(struct SpContextBody *p_ctx) {
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
struct SpResultIntegerUnsigned spBodyCreate(struct SpContextBody *p_ctx) {
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

sp_error_t spBodyDestroy(struct SpContextBody *p_ctx, sp_body_t p_body) {
	spManagerBodyTranslationDestroyEntry(p_ctx->manTrans, p_body);
	return PHYSICS_ERROR_NONE;
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
	man->freed = calloc(g_spBodyDefaultAllocationCount, sizeof(unsigned long long));
	if (!man->freed) {

		free(man->active);
		free(man);
		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	man->capacityFreed = g_spBodyDefaultAllocationCount;
	man->data = calloc(3 * (1 + g_spBodyDefaultAllocationCount), sizeof(struct SpVec3));
	if (!man->data) {

		free(man->active);
		free(man->freed);
		free(man);
		return (struct SpResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	// Add new handles to `man::freed`:
	// for (unsigned long long i = g_spBodyDefaultAllocationCount - 1; i > 0; --i) {
	for (unsigned long long i = 0; i < g_spBodyDefaultAllocationCount; ++i) {

		man->freed[i] = i;

	}

	man->countFreed = g_spBodyDefaultAllocationCount;
	man->countActive = 0;

	return (struct SpResultPointer) { .bad = 0, .result.value = man };
}

sp_error_t spManagerBodyTranslationFree(struct SpManagerBodyTranslation *p_man) {
	if (!p_man) {

		return PHYSICS_ERROR_OBJECT_NULL;

	}

	free(p_man->active);
	free(p_man->freed);
	free(p_man->data);
	free(p_man);

	return PHYSICS_ERROR_NONE;
}

sp_error_t spManagerBodyTranslationCreateEntry(struct SpManagerBodyTranslation *p_man, sp_body_t p_body) {
	if (p_man->countFreed > 0) { // Grab body from free-list.

		p_man->countFreed--;
		p_body = p_man->freed[p_man->countFreed];

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

		void *data = realloc(p_man->data, 2 * 3 * sizeof(struct SpVec3) * (1 + p_man->capacityActive + p_man->capacityFreed));

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
	if (p_man->countFreed >= p_man->capacityFreed) {

		if (p_man->capacityFreed < 1) {

			p_man->capacityFreed = g_spBodyDefaultAllocationCount;

		}

		void *freed = realloc(p_man->freed, 2 * sizeof(unsigned long long) * p_man->capacityFreed);

		if (!freed) {

			// spManagerBodyTranslationFree(p_man); // Yep! Free the *whole* manager.
			return PHYSICS_ERROR_OUT_OF_MEMORY;

		}

		p_man->freed = freed;
		p_man->capacityFreed *= 2;

	}

	for (unsigned long long i = 0; i < p_man->countActive; ++i) {

		if (p_man->active[i] == p_body) {

			p_man->active[i] = p_man->active[p_man->countActive - 1];
			p_man->countActive--;
			p_man->freed[p_man->countFreed] = p_body;
			p_man->countFreed++;

			return PHYSICS_ERROR_NONE;

		}

	}

	return PHYSICS_ERROR_OBJECT_ABSENT;
}
#pragma endregion

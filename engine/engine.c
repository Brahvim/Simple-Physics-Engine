#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "engine/engine.h"

unsigned long long g_physicsBodyDefaultAllocationCount = 16;

#pragma region Solvers.
void physicsSolverTranslationEuler(struct PhysicsManagerBodyTranslation *p_man, float p_dt) {
	for (unsigned long long i = 0; i < 3 * p_man->capacityActive; i += 3) {

		struct PhysicsVec3 *pos = p_man->data + i;
		struct PhysicsVec3 *vel = pos + 1;
		struct PhysicsVec3 *acc = vel + 1;

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

void physicsSolverTranslationVerlet(struct PhysicsManagerBodyTranslation *p_man, float p_dt) {
	for (unsigned long long i = 0; i < 3 * p_man->capacityActive; i += 3) {

		struct PhysicsVec3 *pos = p_man->data + i;
		struct PhysicsVec3 *vel = pos + 1;
		struct PhysicsVec3 *acc = vel + 1;

		pos->x += vel->x * p_dt + (acc->x * 0.5f) * p_dt * p_dt;
		pos->y += vel->y * p_dt + (acc->y * 0.5f) * p_dt * p_dt;
		pos->z += vel->z * p_dt + (acc->z * 0.5f) * p_dt * p_dt;

		vel->x += acc->x * p_dt;
		vel->y += acc->y * p_dt;
		vel->z += acc->z * p_dt;

		acc->x = 0;
		acc->y = 0;
		acc->z = 0;

	}
}
#pragma endregion

#pragma region `struct PhysicsContextBody`.
struct PhysicsResultPointer physicsContextBodyAlloc() {
	struct PhysicsContextBody *ctx = malloc(sizeof(struct PhysicsContextBody));

	if (!ctx) {

		return (struct PhysicsResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	// ctx->masses = calloc(g_physicsBodyDefaultAllocationCount, sizeof(float));
	ctx->manTrans = physicsManagerBodyTranslationAlloc().result.value; // NOLINT clang-analyzer.unix.Malloc
	ctx->capacityMasses = g_physicsBodyDefaultAllocationCount;
	ctx->maxId = 0;

	return (struct PhysicsResultPointer) { .bad = 0, .result.value = ctx }; // cppcheck-suppress unmatchedSuppression
}

physics_error_t physicsContextBodyFree(struct PhysicsContextBody *p_ctx) {
	if (!p_ctx) {

		return PHYSICS_ERROR_OBJECT_NULL;

	}

	physicsManagerBodyTranslationFree(p_ctx->manTrans);
	// free(p_ctx->masses);
	free(p_ctx);

	return PHYSICS_ERROR_NONE;
}
#pragma endregion

#pragma region Bodies!
struct PhysicsResultIntegerUnsigned physicsBodyCreate(struct PhysicsContextBody *p_ctx) {
	physics_body_t const id = p_ctx->maxId;

	physicsManagerBodyTranslationCreateEntry(p_ctx->manTrans, id);

	if (p_ctx->maxId >= p_ctx->capacityMasses) {

		if (p_ctx->capacityMasses < 1) {

			p_ctx->capacityMasses = g_physicsBodyDefaultAllocationCount;

		}

		// float *masses = realloc(p_ctx->masses, 2 * sizeof(float) * p_ctx->capacityMasses);
		//
		// if (!masses) {
		//
		// return (struct PhysicsResultIntegerUnsigned) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };
		//
		// }
		//
		// p_ctx->masses = masses;
		// p_ctx->capacityMasses *= 2;

	}

	// if (id >= p_ctx->capacityMasses) {
	//
	// puts("BUG!");
	//
	// }

	p_ctx->maxId++;
	// p_ctx->masses[id] = 0;
	return (struct PhysicsResultIntegerUnsigned) { .bad = 0, .result.value = id };
}

physics_error_t physicsBodyDestroy(struct PhysicsContextBody *p_ctx, physics_body_t p_body) {
	physicsManagerBodyTranslationDestroyEntry(p_ctx->manTrans, p_body);
	return PHYSICS_ERROR_NONE;
}
#pragma endregion

#pragma region `struct PhysicsManagerBodyTranslation`.
struct PhysicsResultPointer physicsManagerBodyTranslationAlloc() {
	struct PhysicsManagerBodyTranslation *man = malloc(sizeof(struct PhysicsManagerBodyTranslation)); // We zero *everything* later...
	if (!man) {

		return (struct PhysicsResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	man->active = calloc(g_physicsBodyDefaultAllocationCount, sizeof(unsigned long long));
	if (!man->active) {

		free(man);
		return (struct PhysicsResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	man->capacityActive = g_physicsBodyDefaultAllocationCount;
	man->freed = calloc(g_physicsBodyDefaultAllocationCount, sizeof(unsigned long long));
	if (!man->freed) {

		free(man->active);
		free(man);
		return (struct PhysicsResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	man->capacityFreed = g_physicsBodyDefaultAllocationCount;
	man->data = calloc(3 * (1 + g_physicsBodyDefaultAllocationCount), sizeof(struct PhysicsVec3));
	if (!man->data) {

		free(man->active);
		free(man->freed);
		free(man);
		return (struct PhysicsResultPointer) { .bad = 1, .result.error = PHYSICS_ERROR_OUT_OF_MEMORY };

	}

	// Add new handles to `man::freed`:
	// for (unsigned long long i = g_physicsBodyDefaultAllocationCount - 1; i > 0; --i) {
	for (unsigned long long i = 0; i < g_physicsBodyDefaultAllocationCount; ++i) {

		man->freed[i] = i;

	}

	man->countFreed = g_physicsBodyDefaultAllocationCount;
	man->countActive = 0;

	return (struct PhysicsResultPointer) { .bad = 0, .result.value = man };
}

physics_error_t physicsManagerBodyTranslationFree(struct PhysicsManagerBodyTranslation *p_man) {
	if (!p_man) {

		return PHYSICS_ERROR_OBJECT_NULL;

	}

	free(p_man->active);
	free(p_man->freed);
	free(p_man->data);
	free(p_man);

	return PHYSICS_ERROR_NONE;
}

physics_error_t physicsManagerBodyTranslationCreateEntry(struct PhysicsManagerBodyTranslation *p_man, physics_body_t p_body) {
	if (p_man->countFreed > 0) { // Grab body from free-list.

		p_man->countFreed--;
		p_body = p_man->freed[p_man->countFreed];

	}

	if (p_man->countActive >= p_man->capacityActive) {

		if (p_man->capacityActive < 1) {

			p_man->capacityActive = g_physicsBodyDefaultAllocationCount;

		}

		void *active = realloc(p_man->active, 2 * sizeof(unsigned long long) * p_man->capacityActive);

		if (!active) {

			// physicsManagerBodyTranslationFree(p_man); // Yep! Free the *whole* manager.
			return PHYSICS_ERROR_OUT_OF_MEMORY;

		}

		p_man->active = active;
		p_man->capacityActive *= 2;

		void *data = realloc(p_man->data, 2 * 3 * sizeof(struct PhysicsVec3) * (1 + p_man->capacityActive + p_man->capacityFreed));

		if (!data) {

			// physicsManagerBodyTranslationFree(p_man); // Yep! Free the *whole* manager.
			return PHYSICS_ERROR_OUT_OF_MEMORY;

		}

		p_man->data = data;

	}

	memset(&p_man->data[p_body], 0, 3 * sizeof(struct PhysicsVec3)); // NOLINT clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling
	p_man->active[p_man->countActive] = p_body;
	p_man->countActive++;

	return PHYSICS_ERROR_NONE;
}

physics_error_t physicsManagerBodyTranslationDestroyEntry(struct PhysicsManagerBodyTranslation *p_man, physics_body_t p_body) {
	if (p_man->countFreed >= p_man->capacityFreed) {

		if (p_man->capacityFreed < 1) {

			p_man->capacityFreed = g_physicsBodyDefaultAllocationCount;

		}

		void *freed = realloc(p_man->freed, 2 * sizeof(unsigned long long) * p_man->capacityFreed);

		if (!freed) {

			// physicsManagerBodyTranslationFree(p_man); // Yep! Free the *whole* manager.
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

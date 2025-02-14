#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "engine/engine.h"

unsigned long long g_physicsBodyDefaultAllocationCount = 1;

physics_body_t physicsBodyCreate(struct PhysicsBodyContext p_ctx) {
	return 0;
}

void physicsBodyDestroy(struct PhysicsBodyContext p_ctx, physics_body_t p_body) {
}

struct PhysicsManagerBodyTranslation* physicsManagerBodyTranslationAlloc() {
	struct PhysicsManagerBodyTranslation *man = malloc(sizeof(struct PhysicsManagerBodyTranslation)); // We zero *everything* later...
	if (!man) {

		return NULL;

	}

	man->active = calloc(g_physicsBodyDefaultAllocationCount, sizeof(unsigned long long));
	if (!man->active) {

		free(man);
		return NULL;

	}

	man->capacityActive = g_physicsBodyDefaultAllocationCount;
	man->freed = calloc(g_physicsBodyDefaultAllocationCount, sizeof(unsigned long long));
	if (!man->freed) {

		free(man->active);
		free(man);
		return NULL;

	}

	man->capacityFreed = g_physicsBodyDefaultAllocationCount;
	man->data = calloc(g_physicsBodyDefaultAllocationCount, sizeof(struct PhysicsVec3));
	if (!man->data) {

		free(man->active);
		free(man->freed);
		free(man);
		return NULL;

	}

	// Add new handles to `man::freed`:
	for (unsigned long long i = g_physicsBodyDefaultAllocationCount - 1; i > 0; --i) {

		man->freed[i] = i;

	}

	man->countFreed = g_physicsBodyDefaultAllocationCount;
	man->capacityActive = 0;
	man->countActive = 0;

	return man;
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

physics_error_t physicsManagerBodyTranslationCreateEntry(struct PhysicsManagerBodyTranslation *p_man) {
	physics_body_t id = p_man->countActive;

	if (p_man->countFreed > 0) { // Grab body from free-list.

		p_man->countFreed--;
		id = p_man->freed[p_man->countFreed];

	}

	if (p_man->countActive >= p_man->capacityActive) {

		if (p_man->capacityActive < 1) {

			p_man->capacityActive = g_physicsBodyDefaultAllocationCount;

		}

		void *active = realloc(p_man->active, 2 * p_man->capacityActive * sizeof(unsigned long long));

		if (!active) {

			// physicsManagerBodyTranslationFree(p_man); // Yep! Free the *whole* manager.
			return PHYSICS_ERROR_OUT_OF_MEMORY;

		}

		p_man->active = active;
		p_man->capacityActive *= 2;

		void *data = realloc(p_man->data, 2 * (p_man->capacityActive + p_man->capacityFreed) * sizeof(struct PhysicsVec3));

		if (!data) {

			// physicsManagerBodyTranslationFree(p_man); // Yep! Free the *whole* manager.
			return PHYSICS_ERROR_OUT_OF_MEMORY;

		}

		// id = p_man->countActive;
		p_man->data = data;

	}

	p_man->active[p_man->countActive] = id;
	p_man->countActive++;

	// void *addr_max = &p_man->data[p_man->capacityActive];
	// void *addr = &p_man->data[id];
	// printf("%p\n", addr_max);
	// memset(addr, 0, 3 * sizeof(struct PhysicsVec3)); // Remember, `struct PhysicsVec3 PhysicsManagerBodyTranslation::data` holds `(p,v,a)` tuples!
	p_man->data[id].x = 0;
	p_man->data[id].y = 0;
	p_man->data[id].z = 0;

	return PHYSICS_ERROR_NONE;
}

physics_error_t physicsManagerBodyTranslationDestroyEntry(struct PhysicsManagerBodyTranslation *p_man, physics_body_t p_body) {
	if (p_man->countFreed >= p_man->capacityFreed) {

		if (p_man->capacityFreed < 1) {

			p_man->capacityFreed = g_physicsBodyDefaultAllocationCount;

		}

		void *freed = realloc(p_man->freed, 2 * p_man->capacityFreed * sizeof(unsigned long long));

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

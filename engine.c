#include <stdlib.h>
#include <memory.h>

#include "engine.h"

unsigned long long g_physicsBodyDefaultAllocationCount = 0;

physics_body_t physicsBodyCreate(struct PhysicsBodyContext p_ctx) {
	return 0;
}

void physicsBodyDestroy(struct PhysicsBodyContext p_ctx, physics_body_t p_body) {
}

struct PhysicsManagerBodyTranslation* physicsManagerBodyTranslationCreate() {
	struct PhysicsManagerBodyTranslation *const man = calloc(1, sizeof(struct PhysicsManagerBodyTranslation));
	if (!man) {

		return NULL;

	}

	man->active = malloc(man->capacityActive = g_physicsBodyDefaultAllocationCount * sizeof(unsigned long long));
	if (!man->active) {

		free(man);
		return NULL;

	}

	man->freed = malloc(man->capacityFreed = g_physicsBodyDefaultAllocationCount * sizeof(unsigned long long));
	if (!man->freed) {

		free(man->active);
		free(man);
		return NULL;

	}

	man->data = malloc(g_physicsBodyDefaultAllocationCount * sizeof(struct PhysicsVec3));
	if (!man->data) {

		free(man->active);
		free(man->freed);
		free(man);
		return NULL;

	}

	for (unsigned long long i = 0; i < g_physicsBodyDefaultAllocationCount; ++i) {

		man->freed[i] = i;

	}

	man->countFreed = g_physicsBodyDefaultAllocationCount;
	return man;
}

void physicsManagerBodyTranslationDestroy(struct PhysicsManagerBodyTranslation *p_man) {
	if (!p_man) {

		return;

	}

	free(p_man->active);
	free(p_man->freed);
	free(p_man->data);
	free(p_man);
}

void physicsManagerBodyTranslationCreateEntry(struct PhysicsManagerBodyTranslation *p_man, physics_body_t p_body) {
	physics_body_t id = p_man->countActive;

	if (p_man->countFreed > 0) { // Grab body from free-list.

		p_man->countFreed--;
		id = p_man->freed[p_man->countFreed];

	}

	if (p_man->countActive >= p_man->capacityActive) {

		void *active = realloc(p_man->active, 2 * p_man->capacityActive * sizeof(unsigned long long));

		if (!active) {


			physicsManagerBodyTranslationDestroy(p_man); // Yep! Free the *whole* manager.
			return;

		}

		p_man->active = active;
		p_man->capacityActive *= 2;

		void *data = realloc(p_man->data, 2 * p_man->capacityActive * sizeof(struct PhysicsVec3));

		if (!data) {

			physicsManagerBodyTranslationDestroy(p_man); // Yep! Free the *whole* manager.
			return;

		}

		// id = p_man->countActive;
		p_man->data = data;

	}

	p_man->countActive++;
	p_man->active[id] = id;
	memset(&p_man->data[id], 0, 3 * sizeof(struct PhysicsVec3)); // Remember, `struct PhysicsVec3 PhysicsManagerBodyTranslation::data` holds `(p,v,a)` tuples!
}

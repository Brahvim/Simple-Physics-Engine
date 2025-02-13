#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "include/jni/com_brahvim_physics_Engine.h"

#define ITR 100

JNIEXPORT void JNICALL Java_com_brahvim_physics_Engine_create(JNIEnv *p_env, jclass p_class) {
	puts("JVM called `Engine::create()`.");
}

struct Vec3 {

	float x, y, z;

};

struct ManagerBodyTranslation {

	struct Vec3 *data;  // AoS: (pos, vel, acc) packed together

	size_t capacityActive;
	size_t capacityFree;
	size_t countActive;
	size_t countFree;

	size_t *active;
	size_t *free;

};

// ✅ **Preallocates memory** (if needed)
void managerBodyTranslationalCreate(struct ManagerBodyTranslation *man, size_t initialCapacity) {
	if (!man) {

		return;

	}

	man->capacityActive = initialCapacity;
	man->capacityFree = initialCapacity;

	man->countActive = 0;
	man->countFree = initialCapacity;

	// ✅ Allocate memory in one go, avoiding tiny allocations

	// cppcheck-suppress cstyleCast
	man->data = (struct Vec3*) calloc(initialCapacity * 3, sizeof(struct Vec3));  // (pos, vel, acc)

	// cppcheck-suppress cstyleCast
	man->active = (size_t*) calloc(initialCapacity, sizeof(size_t));

	// cppcheck-suppress cstyleCast
	man->free = (size_t*) calloc(initialCapacity, sizeof(size_t));

	// ✅ Initialize free list
	for (size_t i = 0; i < initialCapacity; ++i) {

		man->free[i] = i;

	}
}

void managerBodyTranslationalFree(struct ManagerBodyTranslation *man, size_t id) {
	if (id >= man->capacityActive) {

		fprintf(stderr, "Invalid free: ID %zu out of bounds!\n", id);
		return;

	}

	if (man->countActive == 0) {

		perror("Warning: Trying to free from an empty active list!\n");
		return;

	}

	// ✅ Remove from active list
	int found = 0;
	for (size_t i = 0; i < man->countActive; i++) {

		if (man->active[i] == id) {

			man->active[i] = man->active[--man->countActive];
			found = 1;
			break;

		}

	}

	if (!found) {

		fprintf(stderr, "Warning: Tried to free ID %zu but it wasn't active!\n", id);
		return;

	}

	// ✅ Return to free list only if it’s actually freed
	man->free[man->countFree++] = id;
	// printf("Free ID %zu successfully.\n", id);
}

// ✅ **Allocates a new body from the free list**
size_t managerBodyTranslationalAllocate(struct ManagerBodyTranslation *man) {
	if (man->countFree == 0) {

		// **Grow strategy: Double capacity**
		size_t const newCapacity = man->capacityActive * 2;
		printf("Growing to %zu bodies\n", newCapacity);

		// cppcheck-suppress cstyleCast
		struct Vec3 *const newData = (struct Vec3*) realloc(man->data, 3 * sizeof(struct Vec3) * newCapacity);

		// cppcheck-suppress cstyleCast
		size_t *const newActive = (size_t*) realloc(man->active, sizeof(size_t) * newCapacity);

		// cppcheck-suppress cstyleCast
		size_t *const newFree = (size_t*) realloc(man->free, sizeof(size_t) * newCapacity);

		if (!(newData && newActive && newFree)) {

			perror("Memory allocation failed!\n");
			exit(1);
			return -1; // cppcheck-suppress memleak

		}

		man->data = newData;
		man->free = newFree;
		man->active = newActive;

		// ✅ Update free list with new indices
		for (size_t i = man->capacityActive; i < newCapacity; ++i) {

			man->free[i - man->capacityActive] = i;

		}

		man->capacityFree = newCapacity - man->capacityActive;
		man->countFree = man->capacityFree;
		man->capacityActive = newCapacity;

	}

	// ✅ Allocate from free list
	size_t const id = man->free[man->countFree - 1];
	man->active[man->countActive] = id;
	man->countActive++;
	man->countFree--;

	return id;
}

// ✅ **Cleanup function**
void managerBodyTranslationalDestroy(struct ManagerBodyTranslation *man) {
	free(man->data);
	free(man->free);
	free(man->active);
}

int main() {
	struct ManagerBodyTranslation manager;
	managerBodyTranslationalCreate(&manager, ITR);

	size_t max = ITR;

	printf("Allocating bodies...\n");
	for (int i = 0; i < max; ++i) {

		managerBodyTranslationalAllocate(&manager);
		// printf("Allocated ID: %zu\n", managerBodyTranslationalAllocate(&manager));

	}

	max = ITR / 2;
	for (int i = 0; i < max; ++i) {

		managerBodyTranslationalFree(&manager, i);
		// printf("Allocated ID: %zu\n", managerBodyTranslationalAllocate(&manager));

	}

	printf("Freeing body 5...\n");
	managerBodyTranslationalFree(&manager, 5);

	printf("Allocating new body...\n");
	printf("Allocated ID: %zu\n", managerBodyTranslationalAllocate(&manager));

	max = 2 * ITR;
	for (int i = 0; i < max; ++i) {

		managerBodyTranslationalAllocate(&manager);
		// printf("Allocated ID: %zu\n", managerBodyTranslationalAllocate(&manager));

	}

	managerBodyTranslationalDestroy(&manager);
	return 0;
}

#include <stdio.h>

#include "engine/engine.h"

#define ITR 1000000

int main(int argc, char const *argv[]) {
	puts("Allocating `PhysicsManagerBodyTranslation`.");
	struct PhysicsManagerBodyTranslation *man = physicsManagerBodyTranslationAlloc();

	for (size_t i = 0; i < ITR; i++) {

		physicsManagerBodyTranslationCreateEntry(man);

	}

	for (size_t i = 0; i < ITR; i++) {

		physicsManagerBodyTranslationDestroyEntry(man, i);

	}

	puts("Freeing `PhysicsManagerBodyTranslation`.");
	physicsManagerBodyTranslationFree(man);

	return 0;
}

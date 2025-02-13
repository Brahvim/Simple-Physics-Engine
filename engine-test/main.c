#include <stdio.h>

#include "engine/engine.h"

int main(int argc, char const *argv[]) {

	puts("Allocating `PhysicsManagerBodyTranslation`.");
	struct PhysicsManagerBodyTranslation *man = physicsManagerBodyTranslationCreate();

	puts("Freeing `PhysicsManagerBodyTranslation`.");
	physicsManagerBodyTranslationDestroy(man);

	return 0;
}

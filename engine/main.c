#include "engine/engine.h"

int main(int argc, char const *argv[]) {

	struct PhysicsManagerBodyTranslation *man = physicsManagerBodyTranslationCreate();
	physicsManagerBodyTranslationDestroy(man);

	return 0;
}

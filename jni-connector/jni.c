#include "engine/engine.h"
#include "com_brahvim_physics_Engine.h"

JNIEXPORT void JNICALL Java_com_brahvim_physics_Engine_create(JNIEnv*, jclass) {
	puts("JVM called `Engine::create()`.");

	// puts("Allocating `PhysicsManagerBodyTranslation`.");
	// struct PhysicsManagerBodyTranslation *man = physicsManagerBodyTranslationAlloc().result.value;
	// physicsManagerBodyTranslationFree(man);
	// puts("Freeing `PhysicsManagerBodyTranslation`.");
}

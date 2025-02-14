#include "engine/engine.h"
#include "com_brahvim_physics_Engine.h"

JNIEXPORT void JNICALL Java_com_brahvim_sp_Engine_create(JNIEnv*, jclass) {
	puts("JVM called `Engine::create()`.");

	// puts("Allocating `SpManagerBodyTranslation`.");
	// struct SpManagerBodyTranslation *man = spManagerBodyTranslationAlloc().result.value;
	// spManagerBodyTranslationFree(man);
	// puts("Freeing `SpManagerBodyTranslation`.");
}

#include "ifs.h"
#include "engine/engine.h"
#include "com_brahvim_physics_Engine.h"

JNIEXPORT void JNICALL Java_com_brahvim_sp_Engine_create(JNIEnv *p_env, jclass p_class) {
	puts("JVM called `Engine::create()`.");

	// puts("Allocating `SpContextTranslation`.");
	// struct SpContextTranslation *man = spManagerBodyTranslationAlloc().result.value;
	// spManagerBodyTranslationFree(man);
	// puts("Freeing `SpContextTranslation`.");
}

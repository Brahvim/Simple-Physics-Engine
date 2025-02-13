#include "engine/engine.h"
#include "com_brahvim_physics_Engine.h"

JNIEXPORT void JNICALL Java_com_brahvim_physics_Engine_create(JNIEnv *p_env, jclass p_class) {
    puts("JVM called `Engine::create()`.");

    puts("Allocating `PhysicsManagerBodyTranslation`.");
    struct PhysicsManagerBodyTranslation *man = physicsManagerBodyTranslationCreate();
    physicsManagerBodyTranslationDestroy(man);
    puts("Freeing `PhysicsManagerBodyTranslation`.");
}

#include <stdio.h>
#include <stdlib.h>

#include "engine/engine.h"

#define ITR 10000

int main(int argc, char const *argv[]) {
	puts("Allocating `PhysicsManagerBodyTranslation`.");

	physics_body_t *bodies = malloc(ITR * sizeof(physics_body_t));
	struct PhysicsContextBody *ctx = physicsContextBodyAlloc().result.value;

	for (size_t i = 0; i < ITR; ++i) {

		bodies[i] = physicsBodyCreate(ctx).result.value;

	}

	for (size_t i = 0; i < ITR; ++i) {

		if (i % 3 == 0) {

			struct PhysicsVec3 *pos = &ctx->manTrans->data[i];
			struct PhysicsVec3 *vel = pos + 1;
			struct PhysicsVec3 *acc = vel + 1;

			vel->x += rand();
			acc->y += 0.01f;

		}

		physicsSolverTranslationEuler(ctx->manTrans, 0.01f);

	}

	for (size_t i = 0; i < ITR; ++i) {

		physicsBodyDestroy(ctx, bodies[i]);

	}

	puts("Freeing `PhysicsManagerBodyTranslation`.");
	physicsContextBodyFree(ctx);
	free(bodies);

	return 0;
}

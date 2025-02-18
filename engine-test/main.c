#include <stdio.h>
#include <stdlib.h>

#include "ifs.h"
#include "engine/engine.h"

#define ITR 1000

int main(int const p_argCount, char const *const *const p_argValues) {
	puts("Allocating `SpContextTranslation`.");

	sp_body_t *bodies = malloc(ITR * sizeof(sp_body_t));
	struct SpContext *ctx = spContextAlloc().result.value;

	// Allocation:
	for (size_t i = 0; i < ITR; ++i) {

		bodies[i] = spBodyCreate(ctx).result.value;

	}

	// Simulation modifications:
	for (size_t i = 0; i < ITR; ++i) {

		spBodySetVelX(ctx, i, rand());

	}

	// Simulation:
	for (size_t i = 0; i < ITR; ++i) {

		spBodyAddAccX(ctx, i, 0.01f);
		// spSolveRotationVerlet(ctx->manTrans, 0.01f);
		// spSolveTranslationEuler(ctx->manTrans, 0.01f);
		spSolveTranslationVerlet(ctx->manTrans, 0.01f);

	}

	// De-allocation:
	for (size_t i = 0; i < ITR; ++i) {

		spBodyDestroy(ctx, bodies[i]);

	}

	puts("Freeing `SpContextTranslation`.");
	spContextFree(ctx);
	free(bodies);

	return 0;
}

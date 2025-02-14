#include <stdio.h>
#include <stdlib.h>

#include "engine/engine.h"

#define ITR 100

int main(int argc, char const *argv[]) {
	puts("Allocating `SpManagerBodyTranslation`.");

	sp_body_t *bodies = malloc(ITR * sizeof(sp_body_t));
	struct SpContextBody *ctx = spContextBodyAlloc().result.value;

	for (size_t i = 0; i < ITR; ++i) {

		bodies[i] = spBodyCreate(ctx).result.value;

	}

	for (size_t i = 0; i < ITR; ++i) {

		if (i % 3 == 0) {

			struct SpVec3 *pos = &ctx->manTrans->data[i];
			struct SpVec3 *vel = pos + 1;
			struct SpVec3 *acc = vel + 1;

			vel->x += rand();
			acc->y += 0.01f;

		}

		spSolverTranslationVerlet(ctx->manTrans, 0.01f);

	}

	for (size_t i = 0; i < ITR; ++i) {

		spBodyDestroy(ctx, bodies[i]);

	}

	puts("Freeing `SpManagerBodyTranslation`.");
	spContextBodyFree(ctx);
	free(bodies);

	return 0;
}

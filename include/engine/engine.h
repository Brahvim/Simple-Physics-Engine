#pragma once

typedef unsigned long long sp_body_t;
typedef unsigned long long sp_error_t;

#define PHYSICS_ERROR_NONE 					1 << 0
#define PHYSICS_ERROR_OBJECT_NULL			1 << 1
#define PHYSICS_ERROR_OBJECT_ABSENT			1 << 2
#define PHYSICS_ERROR_OUT_OF_MEMORY			1 << 4

struct SpVec3 {

	float x, y, z;

};

struct SpContextBody {

	struct SpManagerBodyTranslation {

		unsigned long long capacityActive;
		unsigned long long capacityFreed;
		unsigned long long countActive;
		unsigned long long countFreed;

		unsigned long long *active;
		unsigned long long *freed;

		struct SpVec3 *data; // AoS with `(pos, vel, acc)` tuples.

	} *manTrans;
	unsigned long long capacityMasses;
	unsigned long long maxId;
	float *masses;

};

struct SpResultFloat {

	char bad;

	union SpUnionErrorFloat {

		float *value;
		sp_error_t error;

	} result;


};

struct SpResultPointer {

	char bad;

	union SpUnionErrorPointer {

		void *value;
		sp_error_t error;

	} result;


};

struct SpResultIntegerSigned {

	char bad;

	union SpUnionErrorIntegerSigned {

		sp_error_t error;
		signed long long value;

	} result;


};

struct SpResultIntegerUnsigned {

	char bad;

	union SpUnionErrorUll {

		sp_error_t error;
		unsigned long long value;

	} result;


};

extern unsigned long long g_spBodyDefaultAllocationCount;

struct SpResultPointer spContextBodyAlloc();
sp_error_t spContextBodyFree(struct SpContextBody *ctx);

// void spBodyForceCenter();

struct SpResultIntegerUnsigned spBodyCreate(struct SpContextBody *ctx);
sp_error_t spBodyDestroy(struct SpContextBody *ctx, sp_body_t body);

void spSolverTranslationEuler(struct SpManagerBodyTranslation *man, float dt);
void spSolverTranslationVerlet(struct SpManagerBodyTranslation *man, float dt);

struct SpResultPointer spManagerBodyTranslationAlloc();
sp_error_t spManagerBodyTranslationFree(struct SpManagerBodyTranslation* man);
sp_error_t spManagerBodyTranslationCreateEntry(struct SpManagerBodyTranslation* man, sp_body_t body);
sp_error_t spManagerBodyTranslationDestroyEntry(struct SpManagerBodyTranslation* man, sp_body_t body);

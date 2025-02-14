#pragma once

typedef unsigned long long physics_body_t;
typedef unsigned long long physics_error_t;

#define PHYSICS_ERROR_NONE 					1 << 0
#define PHYSICS_ERROR_OBJECT_NULL			1 << 1
#define PHYSICS_ERROR_OBJECT_ABSENT			1 << 2
#define PHYSICS_ERROR_OUT_OF_MEMORY			1 << 4

struct PhysicsVec3 {

	float x, y, z;

};

struct PhysicsContextBody {

	struct PhysicsManagerBodyTranslation {

		unsigned long long capacityActive;
		unsigned long long capacityFreed;
		unsigned long long countActive;
		unsigned long long countFreed;

		unsigned long long *active;
		unsigned long long *freed;

		struct PhysicsVec3 *data; // AoS with `(pos, vel, acc)` tuples.

	} *manTrans;
	unsigned long long capacityMasses;
	unsigned long long maxId;
	float *masses;

};

struct PhysicsResultFloat {

	char bad;

	union PhysicsUnionErrorFloat {

		float *value;
		physics_error_t error;

	} result;


};

struct PhysicsResultPointer {

	char bad;

	union PhysicsUnionErrorPointer {

		void *value;
		physics_error_t error;

	} result;


};

struct PhysicsResultIntegerSigned {

	char bad;

	union PhysicsUnionErrorIntegerSigned {

		physics_error_t error;
		signed long long value;

	} result;


};

struct PhysicsResultIntegerUnsigned {

	char bad;

	union PhysicsUnionErrorUll {

		physics_error_t error;
		unsigned long long value;

	} result;


};

extern unsigned long long g_physicsBodyDefaultAllocationCount;

struct PhysicsResultPointer physicsContextBodyAlloc();
physics_error_t physicsContextBodyFree(struct PhysicsContextBody *ctx);

// void physicsBodyForceCenter();

struct PhysicsResultIntegerUnsigned physicsBodyCreate(struct PhysicsContextBody *ctx);
physics_error_t physicsBodyDestroy(struct PhysicsContextBody *ctx, physics_body_t body);

void physicsSolverTranslationEuler(struct PhysicsManagerBodyTranslation *man, float dt);
void physicsSolverTranslationVerlet(struct PhysicsManagerBodyTranslation *man, float dt);

struct PhysicsResultPointer physicsManagerBodyTranslationAlloc();
physics_error_t physicsManagerBodyTranslationFree(struct PhysicsManagerBodyTranslation* man);
physics_error_t physicsManagerBodyTranslationCreateEntry(struct PhysicsManagerBodyTranslation* man, physics_body_t body);
physics_error_t physicsManagerBodyTranslationDestroyEntry(struct PhysicsManagerBodyTranslation* man, physics_body_t body);

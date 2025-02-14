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

struct PhysicsManagerBodyTranslation {

	unsigned long long capacityActive;
	unsigned long long capacityFreed;
	unsigned long long countActive;
	unsigned long long countFreed;

	unsigned long long *active;
	unsigned long long *freed;

	struct PhysicsVec3 *data; // AoS with `(pos, vel, acc)` tuples.

};

struct PhysicsBodyContext {

	struct PhysicsManagerBodyTranslation manTrans;

};

extern unsigned long long g_physicsBodyDefaultAllocationCount;

physics_body_t physicsBodyCreate(struct PhysicsBodyContext ctx);
void physicsBodyDestroy(struct PhysicsBodyContext ctx, physics_body_t body);

struct PhysicsManagerBodyTranslation* physicsManagerBodyTranslationAlloc();
physics_error_t physicsManagerBodyTranslationFree(struct PhysicsManagerBodyTranslation* man);
physics_error_t physicsManagerBodyTranslationCreateEntry(struct PhysicsManagerBodyTranslation* man);
physics_error_t physicsManagerBodyTranslationDestroyEntry(struct PhysicsManagerBodyTranslation* man, physics_body_t body);

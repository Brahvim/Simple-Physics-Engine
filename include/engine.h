#pragma once

typedef unsigned long long physics_body_t;

extern unsigned long long g_physicsBodyDefaultAllocationCount;

enum PhysicsCrashyError {

	PHYSICS_ERROR_BODY_OUT_OF_MEMORY,

};

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

physics_body_t physicsBodyCreate(struct PhysicsBodyContext ctx);
void physicsBodyDestroy(struct PhysicsBodyContext ctx, physics_body_t body);

struct PhysicsManagerBodyTranslation* physicsManagerBodyTranslationCreate();
void physicsManagerBodyTranslationDestroy(struct PhysicsManagerBodyTranslation* man);
void physicsManagerBodyTranslationCreateEntry(struct PhysicsManagerBodyTranslation* man, physics_body_t body);

#pragma once

typedef unsigned long long sp_size_t;
typedef unsigned long long sp_body_t;

extern unsigned long long g_spBodyDefaultAllocationCount;

struct SpQuat {

	float x, y, z, w;

};

struct SpVec2 {

	float x, y;

};

struct SpVec3 {

	float x, y, z;

};

struct SpContext {

	struct SpContextRotation {

		unsigned long long capacityInactive;
		unsigned long long capacityActive;
		unsigned long long countInactive;
		unsigned long long countActive;

		unsigned long long *inactive;
		unsigned long long *active;

		struct SpSolverParametersRotation {

			struct SpVec3 angles;
			struct SpVec3 angularVelocity;
			struct SpVec3 angularAcceleration;

		} *data; // AoS with `(pos, vel, acc)` tuples.

	} *ctxRot;

	struct SpContextTranslation {

		unsigned long long capacityInactive;
		unsigned long long capacityActive;
		unsigned long long countInactive;
		unsigned long long countActive;

		unsigned long long *inactive;
		unsigned long long *active;

		struct SpSolverParametersTranslation {

			struct SpVec3 position;
			struct SpVec3 velocity;
			struct SpVec3 acceleration;

		} *data; // AoS with `(pos, vel, acc)` tuples.

	} *ctxTrans;

	unsigned long long capacityMasses;
	unsigned long long maxId;
	float *masses;

};

struct SpContext* spContextAlloc();
void spContextFree(struct SpContext *const restrict ctx);
sp_body_t spBodyCreate(struct SpContext *const restrict ctx);
void spBodyDestroy(struct SpContext *const restrict ctx, sp_body_t const body);

// void spSolveRotationEuler(struct SpContextTranslation *const restrict ctx, float dt);
void spSolveTranslationEuler(struct SpContextTranslation *const restrict ctx, float dt);
void spSolveTranslationVerlet(struct SpContextTranslation *const restrict ctx, float dt);

struct SpContextTranslation* spContextTranslationAlloc();
void spContextTranslationFree(struct SpContextTranslation *const restrict ctx);
void spContextTranslationCreateEntry(struct SpContextTranslation *const restrict ctx, sp_body_t const body);
void spContextTranslationDestroyEntry(struct SpContextTranslation *const restrict ctx, sp_body_t const body);

float spBodyGetMass(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetPosX(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetPosY(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetPosZ(struct SpContext *const restrict ctx, sp_body_t const body);
struct SpVec3* spBodyGetPosition(struct SpContext *const restrict ctx, sp_body_t const body);

float spBodyGetVelX(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetVelY(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetVelZ(struct SpContext *const restrict ctx, sp_body_t const body);
struct SpVec3* spBodyGetVelocity(struct SpContext *const restrict ctx, sp_body_t const body);

float spBodyGetAccX(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetAccY(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetAccZ(struct SpContext *const restrict ctx, sp_body_t const body);
struct SpVec3* spBodyGetAcceleration(struct SpContext *const restrict ctx, sp_body_t const body);

float spBodyGetAngX(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetAngY(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetAngZ(struct SpContext *const restrict ctx, sp_body_t const body);
struct SpVec3* spBodyGetAngles(struct SpContext *const restrict ctx, sp_body_t const body);

float spBodyGetVelAngX(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetVelAngY(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetVelAngZ(struct SpContext *const restrict ctx, sp_body_t const body);
struct SpVec3* spBodyGetAngularVelocity(struct SpContext *const restrict ctx, sp_body_t const body);

float spBodyGetAccAngX(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetAccAngY(struct SpContext *const restrict ctx, sp_body_t const body);
float spBodyGetAccAngZ(struct SpContext *const restrict ctx, sp_body_t const body);
struct SpVec3* spBodyGetAngularAcceleration(struct SpContext *const restrict ctx, sp_body_t const body);

void spBodySetMass(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetPosX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetPosY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetPosZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetPosition(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodySetVelX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetVelY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetVelZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetVelocity(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodySetAccX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetAccY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetAccZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetAcceleration(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodySetAngX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetAngY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetAngZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetAngles(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodySetVelAngX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetVelAngY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetVelAngZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetVelocityAngular(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodySetAccAngX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetAccAngY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetAccAngZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodySetAccelerationAngular(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodyAddMass(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddPosX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddPosY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddPosZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddPosition(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodyAddVelX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddVelY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddVelZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddVelocity(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodyAddAccX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddAccY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddAccZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddAcceleration(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodyAddAngX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddAngY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddAngZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddAngles(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodyAddVelAngX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddVelAngY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddVelAngZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddVelocityAngular(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodyAddAccAngX(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddAccAngY(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddAccAngZ(struct SpContext *const restrict ctx, sp_body_t const body, float value);
void spBodyAddAccelerationAngular(struct SpContext *const restrict ctx, sp_body_t const body, float x, float y, float z);

void spBodyForceCenter(struct SpContext *const restrict ctx, sp_body_t const body, float forceX, float force, float forceZ);

void spBodyForce(struct SpContext *const restrict ctx, sp_body_t const body, float forceX, float force, float forceZ, float positionX, float positionY, float positionZ);

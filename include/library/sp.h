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
void spContextFree(struct SpContext *restrict ctx);
sp_body_t spBodyCreate(struct SpContext *restrict ctx);
void spBodyDestroy(struct SpContext *restrict ctx, sp_body_t body);

// void spSolveRotationEuler(struct SpContextTranslation *restrict ctx, float dt);
void spSolveTranslationEuler(struct SpContextTranslation *restrict ctx, float dt);
void spSolveTranslationVerlet(struct SpContextTranslation *restrict ctx, float dt);

struct SpContextTranslation* spContextTranslationAlloc();
void spContextTranslationFree(struct SpContextTranslation *restrict ctx);
void spContextTranslationCreateEntry(struct SpContextTranslation *restrict ctx, sp_body_t body);
void spContextTranslationDestroyEntry(struct SpContextTranslation *restrict ctx, sp_body_t body);

float spBodyGetMass(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetPosX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetPosY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetPosZ(struct SpContext *restrict ctx, sp_body_t body);
struct SpVec3* spBodyGetPosition(struct SpContext *restrict ctx, sp_body_t body);

float spBodyGetVelX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetVelY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetVelZ(struct SpContext *restrict ctx, sp_body_t body);
struct SpVec3* spBodyGetVelocity(struct SpContext *restrict ctx, sp_body_t body);

float spBodyGetAccX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAccY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAccZ(struct SpContext *restrict ctx, sp_body_t body);
struct SpVec3* spBodyGetAcceleration(struct SpContext *restrict ctx, sp_body_t body);

float spBodyGetAngX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAngY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAngZ(struct SpContext *restrict ctx, sp_body_t body);
struct SpVec3* spBodyGetAngles(struct SpContext *restrict ctx, sp_body_t body);

float spBodyGetVelAngX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetVelAngY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetVelAngZ(struct SpContext *restrict ctx, sp_body_t body);
struct SpVec3* spBodyGetAngularVelocity(struct SpContext *restrict ctx, sp_body_t body);

float spBodyGetAccAngX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAccAngY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAccAngZ(struct SpContext *restrict ctx, sp_body_t body);
struct SpVec3* spBodyGetAngularAcceleration(struct SpContext *restrict ctx, sp_body_t body);

void spBodySetMass(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetPosX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetPosY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetPosZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetPosition(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodySetVelX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetVelY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetVelZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetVelocity(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodySetAccX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetAccY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetAccZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetAcceleration(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodySetAngX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetAngY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetAngZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetAngles(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodySetVelAngX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetVelAngY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetVelAngZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetVelocityAngular(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodySetAccAngX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetAccAngY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetAccAngZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodySetAccelerationAngular(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodyAddMass(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddPosX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddPosY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddPosZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddPosition(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodyAddVelX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddVelY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddVelZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddVelocity(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodyAddAccX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddAccY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddAccZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddAcceleration(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodyAddAngX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddAngY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddAngZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddAngles(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodyAddVelAngX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddVelAngY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddVelAngZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddVelocityAngular(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodyAddAccAngX(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddAccAngY(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddAccAngZ(struct SpContext *restrict ctx, sp_body_t body, float value);
void spBodyAddAccelerationAngular(struct SpContext *restrict ctx, sp_body_t body, float x, float y, float z);

void spBodyForceCenter(struct SpContext *restrict ctx, sp_body_t body, float forceX, float force, float forceZ);

void spBodyForce(struct SpContext *restrict ctx, sp_body_t body, float forceX, float force, float forceZ, float positionX, float positionY, float positionZ);

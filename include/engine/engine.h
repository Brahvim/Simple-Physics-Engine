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

struct SpContext {

	struct SpManagerBodyTranslation {

		unsigned long long capacityInactive;
		unsigned long long capacityActive;
		unsigned long long countInactive;
		unsigned long long countActive;

		unsigned long long *inactive;
		unsigned long long *active;

		struct SpVec3 *data; // AoS with `(pos, vel, acc)` tuples.

	} *manTrans;

	struct SpManagerBodyRotation {

		unsigned long long capacityInactive;
		unsigned long long capacityActive;
		unsigned long long countInactive;
		unsigned long long countActive;

		unsigned long long *inactive;
		unsigned long long *active;

		struct SpVec3 *data; // AoS with `(pos, vel, acc)` tuples.

	} *manRot;

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
sp_error_t spContextBodyFree(struct SpContext *ctx);

// void spBodyForceCenter();

struct SpResultIntegerUnsigned spBodyCreate(struct SpContext *ctx);
sp_error_t spBodyDestroy(struct SpContext *ctx, sp_body_t body);

void spSolveTranslationEuler(struct SpManagerBodyTranslation *man, float dt);
void spSolveTranslationVerlet(struct SpManagerBodyTranslation *man, float dt);

struct SpResultPointer spManagerBodyTranslationAlloc();
sp_error_t spManagerBodyTranslationFree(struct SpManagerBodyTranslation* man);
sp_error_t spManagerBodyTranslationCreateEntry(struct SpManagerBodyTranslation* man, sp_body_t body);
sp_error_t spManagerBodyTranslationDestroyEntry(struct SpManagerBodyTranslation* man, sp_body_t body);

float spBodyGetMass(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetPosX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetPosY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetPosZ(struct SpContext *restrict ctx, sp_body_t body);
struct SpVec3* spBodyGetPos(struct SpContext *restrict ctx, sp_body_t body);

float spBodyGetVelX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetVelY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetVelZ(struct SpContext *restrict ctx, sp_body_t body);
struct SpVec3* spBodyGetVel(struct SpContext *restrict ctx, sp_body_t body);

float spBodyGetAccX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAccY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAccZ(struct SpContext *restrict ctx, sp_body_t body);
struct SpVec3* spBodyGetAcc(struct SpContext *restrict ctx, sp_body_t body);

float spBodyGetAngX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAngY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAngZ(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetVelAngX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetVelAngY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetVelAngZ(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAccAngX(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAccAngY(struct SpContext *restrict ctx, sp_body_t body);
float spBodyGetAccAngZ(struct SpContext *restrict ctx, sp_body_t body);

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

void spBodyForceCenter(struct SpContext *restrict ctx, sp_body_t body, float forceX, float force, float forceZ);

void spBodyForce(struct SpContext *restrict ctx, sp_body_t body, float forceX, float force, float forceZ, float positionX, float positionY, float positionZ);

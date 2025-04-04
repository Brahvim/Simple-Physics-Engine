#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <immintrin.h>  // SIMD Intrinsics

#define N 10000000  // Number of elements
#define CHUNK_SIZE 8  // AVX processes 8 floats per vector

typedef struct {
	float x, y, z;
} Vec3_AoS;

typedef struct {
	__attribute__((aligned(64))) float x[CHUNK_SIZE];
	__attribute__((aligned(64))) float y[CHUNK_SIZE];
	__attribute__((aligned(64))) float z[CHUNK_SIZE];
} Vec3_SoA_Chunk;

void init_AoS(Vec3_AoS *data) {
	for (int i = 0; i < N; i++) {
		data[i].x = (float) rand() / RAND_MAX;
		data[i].y = (float) rand() / RAND_MAX;
		data[i].z = (float) rand() / RAND_MAX;
	}
}

void euler_AoS(Vec3_AoS *pos, Vec3_AoS *vel, float dt) {
	for (int i = 0; i < N; i++) {
		vel[i].x += 0.1f * dt;
		vel[i].y += 0.1f * dt;
		vel[i].z += 0.1f * dt;

		pos[i].x += vel[i].x * dt;
		pos[i].y += vel[i].y * dt;
		pos[i].z += vel[i].z * dt;
	}
}

void init_SoA(Vec3_SoA_Chunk *data) {
	for (int i = 0; i < N / CHUNK_SIZE; i++) {
		for (int j = 0; j < CHUNK_SIZE; j++) {
			data[i].x[j] = (float) rand() / RAND_MAX;
			data[i].y[j] = (float) rand() / RAND_MAX;
			data[i].z[j] = (float) rand() / RAND_MAX;
		}
	}
}

void euler_SoA_AVX(Vec3_SoA_Chunk *pos, Vec3_SoA_Chunk *vel, float dt) {
	__m256 v_dt = _mm256_set1_ps(dt);
	__m256 v_acc = _mm256_set1_ps(0.1f); // Fake acceleration

	for (int i = 0; i < N / CHUNK_SIZE; i++) {
		// Load velocity
		__m256 vx = _mm256_load_ps(vel[i].x);
		__m256 vy = _mm256_load_ps(vel[i].y);
		__m256 vz = _mm256_load_ps(vel[i].z);

		// Integrate velocity ( v += a * dt )
		vx = _mm256_fmadd_ps(v_acc, v_dt, vx);
		vy = _mm256_fmadd_ps(v_acc, v_dt, vy);
		vz = _mm256_fmadd_ps(v_acc, v_dt, vz);

		// Store velocity
		_mm256_store_ps(vel[i].x, vx);
		_mm256_store_ps(vel[i].y, vy);
		_mm256_store_ps(vel[i].z, vz);

		// Load position
		__m256 px = _mm256_load_ps(pos[i].x);
		__m256 py = _mm256_load_ps(pos[i].y);
		__m256 pz = _mm256_load_ps(pos[i].z);

		// Integrate position ( p += v * dt )
		px = _mm256_fmadd_ps(vx, v_dt, px);
		py = _mm256_fmadd_ps(vy, v_dt, py);
		pz = _mm256_fmadd_ps(vz, v_dt, pz);

		// Store position
		_mm256_store_ps(pos[i].x, px);
		_mm256_store_ps(pos[i].y, py);
		_mm256_store_ps(pos[i].z, pz);
	}
}

int main() {
	Vec3_AoS *pos_AoS = aligned_alloc(64, N * sizeof(Vec3_AoS));
	Vec3_AoS *vel_AoS = aligned_alloc(64, N * sizeof(Vec3_AoS));

	Vec3_SoA_Chunk *pos_SoA = _mm_malloc(N * sizeof(Vec3_SoA_Chunk), 32);
	Vec3_SoA_Chunk *vel_SoA = _mm_malloc(N * sizeof(Vec3_SoA_Chunk), 32);

	init_AoS(pos_AoS);
	init_AoS(vel_AoS);

	init_SoA(pos_SoA);
	init_SoA(vel_SoA);

	float dt = 0.016f;

	struct timespec ts;

	clock_gettime(1, &ts);
	double start = ts.tv_sec + ts.tv_nsec * 1e-9;

	euler_AoS(pos_AoS, vel_AoS, dt);

	clock_gettime(1, &ts);
	double time_AoS = ts.tv_sec + ts.tv_nsec * 1e-9 - start;

	clock_gettime(1, &ts);
	start = ts.tv_sec + ts.tv_nsec * 1e-9;

	euler_SoA_AVX(pos_SoA, vel_SoA, dt);

	clock_gettime(1, &ts);
	double time_SoA_AVX = ts.tv_sec + ts.tv_nsec * 1e-9 - start;

	printf("AoS Time: %.6f sec\n", time_AoS);
	printf("SoA (AVX-Optimized) Time: %.6f sec\n", time_SoA_AVX);
	printf("The winner is %s!\n", time_AoS < time_SoA_AVX ? "the AoS" : "the AVX-based SoA");

	free(pos_AoS);
	free(vel_AoS);

	free(pos_SoA);
	free(vel_SoA);

	return 0;
}

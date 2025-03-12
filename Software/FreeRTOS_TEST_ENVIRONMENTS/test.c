#include <stdio.h>
#include <assert.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>

void *TEST_THREAD1(void) {

}

void *TEST_THREAD2(void) {

}


void *perform_work(void *arguments) {
	int index = 1;
	int sleep_time = 1 + rand() % 5;
	printf("Thread %d: Started.\n", index);
	printf("Thread %d: Will be sleeping for %d seconds.\n", index, sleep_time);
	sleep(sleep_time);
	printf("Thread %d: Ended.\n", index);
	return NULL;
}

int main(void) {
	pthread_t THREADS[5];
	int i;
	int result_code;

	for(int i = 0; i < 5; i++) {
		printf("Thread: %d Made!\n", i);
		result_code = pthread_create(&THREADS[i], NULL, perform_work, NULL);
			// change args past the thread table addr perhaps?
		assert(!result_code);
	}

	printf("Threads created!\n");

	for(int i = 0; i < 5; i++) {
		result_code = pthread_join(THREADS[i], NULL);
		assert(!result_code);
		printf("Thread %d has ended\n", i);
	}

	printf("END!\n");
	return 0;
}

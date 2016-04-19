#include <stdio.h>
#include <stdint.h>


static void sht1x_data(const char* data) {
	uint32_t inst = -1, cnt = -1, temp = -1, hum = -1;
	if(4 != sscanf(data, "%x %x %x %x", &inst, &cnt, &temp, &hum)) {
		return;
	}
	float ftemp = temp;
	float fhum = hum;
	ftemp *= 0.01;
	ftemp -= 40.1;
	fhum = (ftemp - 25.0) * (0.01 + 0.00008 * fhum)
		+ 0.0367 * fhum
		- 1.5955e-6 * fhum * fhum
		- 2.0468;
	printf("SHT1X:\n"
	"\tinstance = %d\n"
	"\tcount    = %d\n"
	"\ttemp     = %.1f\n"
	"\thumidity = %.1f\n",
			inst, cnt, ftemp, fhum);
}

static void (*func_arr[])(const char* data) = {
	sht1x_data
};

int main() {
	char  buff[256];
	while(fgets(buff, sizeof(buff), stdin)) {
		uint32_t func = -1;
		sscanf(buff, "%02x", &func);
		if(func > 0) {
			continue;
		}
		func_arr[func](buff + 3);
	}
	return 0;
}


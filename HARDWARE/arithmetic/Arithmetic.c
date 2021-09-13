#include "stdio.h"
#include "math.h"
#include "string.h"
#include "Arithmetic.h"

/*Ï£¶ûÅÅĞò*/
void shell_sort(unsigned short arr[], int len){
	int gap, i, j;
	int temp;
	for (gap = len >> 1; gap > 0; gap = gap >> 1){
		for (i = gap; i < len; i++) {
			temp = arr[i];
			for (j = i - gap; j >= 0 && arr[j] > temp; j -= gap)
				arr[j + gap] = arr[j];
			arr[j + gap] = temp;
		}
	}
}




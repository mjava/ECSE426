#include <stdio.h>
#include "arm_math.h"
#include <math.h>
#include <stdlib.h>

void FIR_C(int inputVector[], float outputVector[], int inputLength, int outputLength, int order);
void c_math(float inputVector[], float outputVector[], int length);
extern void asm_math(float inputVector[], float outputVector[], int length);
void cmsis_math(float inputVector[], float outputVector[], uint32_t outputIndex[], int length);

int main()
{
	
	int input_vector[100] = {12,21,19,45,34,39,50,52,63,66,83,87,74,92,86,93,99,109,101,115,103,112,117,111,114,113,103,101,117,99,94,102,105,98,85,84,82,88,67,76,67,56,46,45,41,27,31,27,14,12,-1,-7,-6,-20,-14,-17,-28,-41,-42,-57,-46,-51,-56,-72,-69,-84,-79,-84,-90,-92,-88,-96,-87,-90,-86,-86,-86,-98,-96,-98,-82,-77,-80,-68,-67,-58,-62,-62,-62,-47,-38,-40,-41,-32,-28,-19,-10,-2,3,18};
	
	int inputLength = sizeof(input_vector)/sizeof(input_vector[0]);
	int order = 4;
	int outputLength = inputLength-order;
	float *output_vector = malloc(sizeof(outputLength));
	float output_math_vector_c[5] = {0};
	float output_math_vector_asm[5] = {0};
	float output_math_vector_cmsis[5] = {0};
	uint32_t output_math_vector_cmsis_index[2] = {0};
	int output_math_vector_length = 5;
	int i;
	
	printf("Begin FIR filter\n");
	
	FIR_C(input_vector, output_vector, inputLength, outputLength, order);
	
	for(i=0; i < outputLength; i++) {
		printf("The output vector at index %d is: %f\n", i, output_vector[i]);
	}
	
	printf("Begin C calculations\n");
	
	c_math(output_vector, output_math_vector_c, outputLength);
	
	for(i=0; i<output_math_vector_length; i++){
			printf("The outputs are: %f\n", output_math_vector_c[i]);
	}
	
	printf("Begin ASM calculations\n");
	
	asm_math(output_vector, output_math_vector_asm, outputLength);

	for(i=0; i<output_math_vector_length; i++){
			printf("The outputs are: %f\n", output_math_vector_asm[i]);
	}
	
	printf("Begin CMSIS calculations\n");
	
	cmsis_math(output_vector, output_math_vector_cmsis, output_math_vector_cmsis_index, outputLength);

	for(i=0; i<output_math_vector_length; i++){
			printf("The outputs are: %f\n", output_math_vector_cmsis[i]);
	}
	
	printf("The end!\n");
	
	return 0;
}

void FIR_C(int inputVector[], float outputVector[], int inputLength, int outputLength, int order) {
	
	float output;
	int i, j;
	int inputVariable;
	
	float coefficients[5] = {0.2, 0.2, 0.2, 0.2, 0.2};

	//add the sum of inputVector[0]*coeeficient[0]...inputVector[4]*coefficient[4] and save in outputVector[0]
	//then repeat for inputVector[1]*coefficient[0]...inputVector[5]*coefficient[4] and save in outputVector[1]
	//incrementally until inputVector[5]*coefficient[0]...inputVector[9]*coefficient[4] and save in outputVector[5]
	for(i = 0; i < outputLength; i++){
		output = 0;
		
		for(j = 0; j < order+1; j++){
			inputVariable = i + j;
			output += inputVector[inputVariable] * coefficients[j];
		}
		outputVector[i] = output;
	}
}

void c_math(float inputVector[], float outputVector[], int length){
	
	float rms_value = 0;
	float max_value = 0;
	int max_index = 0;
	float min_value = 0;
	int min_index = 0;
	
	float sum = 0;
	
	int i;
	//intiialize max and min values to first input
	max_value = inputVector[0];
	min_value = inputVector[0];

	for(i=0; i<length; i++) {
		//calculate sum of squares
		sum += (float)(pow(inputVector[i],2));
		//at the second input, enter loop to compare inputs for max or min
		if(i>0){
			//if current value is larger then set max value, replace max value with current value
			if(inputVector[i] > max_value){
				max_value = inputVector[i];
				max_index = i;
			}
			//if current value is smaller then set min value, replace min value with current value
			if(inputVector[i] < min_value) {
				min_value = inputVector[i];
				min_index = i;
			}
		}
	}

	//rms is square root of (sum of squares divided by length)
	rms_value = (float) (sqrt(sum/((float)(length))));
	
	outputVector[0] = rms_value;
	outputVector[1] = max_value;
	outputVector[2] = max_index;
	outputVector[3] = min_value;
	outputVector[4] = min_index;
}

void cmsis_math(float inputVector[], float outputVector[], uint32_t outputIndex[], int length) {
	arm_rms_f32(inputVector, length, &outputVector[0]);
	arm_max_f32(inputVector, length, &outputVector[1], &outputIndex[0]);
	outputVector[2] = (float)(outputIndex[0]);
	arm_min_f32(inputVector, length, &outputVector[3], &outputIndex[1]);
	outputVector[4] = (float)(outputIndex[1]);
}
	


/*
 * svm.h
 *
 *  Created on: 2022年9月4日
 *      Author: dztu
 */

#ifndef INC_SVM_H_
#define INC_SVM_H_

#include "math.h"
#include <stdlib.h>

typedef struct svm_model
{
//	char* svm_type[10];
//	char* kernel_type[10];

	float gamma;			/* gamma in RBF kernel */
	int n_class;			/* number of classes*/
	int total_n_sv;			/* total #SV */
	float *intercept;		/* constants in decision functions (rho[k*(k-1)/2]) */
	int *label;				/* label of each class (label[k]) */
	int *n_sv;				/* number of SVs for each class (nSV[k]);  nSV[0] + nSV[1] + ... + nSV[k-1] = l */
	int n_feature;
	float *scaler_min;
	float *scaler_max;

	float **SV;				/* SVs (SV[l]) */
	float **dual_coef;		/* coefficients for SVs in decision functions (sv_coef[k-1][l]) */

} svm_model;


float rbfKernel(float *x1, float *x2, int size, float gamma);
int svmPredict(float *x, svm_model *model);


#endif /* INC_SVM_H_ */

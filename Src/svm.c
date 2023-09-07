/*
 * svm.c
 *
 *  Created on: Sep.04, 2022
 *     Author: Dezhan Tu
 *     Version: 1.0
 */

#include "svm.h"


/**
 * RBF Kernel Function
 * measure the similarity of two vector in L2 distance
 */
float rbfKernel(float *x1, float *x2, int size, float gamma)
{
	float acc=0, tmp;
	for(int i=0; i<size; i++){
		tmp = x1[i] - x2[i];
		acc += tmp * tmp;
	}
	return expf(-gamma * acc);
}


/**
 * SVM Prediction based on OVO
 */

int svmPredict(float *x, svm_model *model)
{

	int i, j, k, result;
	int n_class = model->n_class;
	int total_n_sv = model->total_n_sv;  										/* total #SV */

	float kvalue[total_n_sv];
	for(i=0; i<total_n_sv;i++)
		kvalue[i] = rbfKernel(x, model->SV[i], model->n_feature, model->gamma);

	/*************Write your code start***************/

	// implement more code for multi-class SVM Prediction

    // implement more code for multi-class SVM Prediction
    int start[n_class];
    for(i = 0; i < n_class; i++) {
        start[i] = 0;
    }
    for(i = 1; i < n_class; i++) {
        start[i] = start[i-1] + model->n_sv[i-1];
    }

    float vote[n_class];
    for(i = 0; i < n_class; i++) {
        vote[i] = 0;
    }

    int p, si, sj, ci, cj = 0;
    float acc;
    float * coef1;
	float * coef2;
	p = 0;
    for(i = 0; i < n_class; i++) {
        for(j = i+1; j < n_class; j++) {
            acc = 0;
        	si = start[i];
            sj = start[j];
            ci = model->n_sv[i];
            cj = model->n_sv[j];
            coef1 = model->dual_coef[j-1];
            coef2 = model->dual_coef[i];

            for(k = 0; k < ci; k++) {
                acc = acc + coef1[si + k] * kvalue[si + k];
            }
            for(k = 0; k < cj; k++) {
                acc = acc + coef2[sj + k] * kvalue[sj + k];
            }

            acc = acc + model->intercept[p];
            if(acc > 0) {
                vote[i] = vote[i] + 1;
            }
            else{
                vote[j] = vote[j] + 1;
            }
            p=p+1;
        }
    }

    int vote_max_idx = 0;
    for(i = 0; i < n_class; i++){
        if(vote[i] > vote[vote_max_idx]) {
            vote_max_idx = i;
        }
    }
    result = vote_max_idx;

    return result;

	/*************Write your code End***************/

}


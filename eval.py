#!/usr/bin/env python3

import numpy as np
import sys

TEST_DIR = "./"
GT_DIR = 'labeled/'

def get_mse(gt, test):
  test = np.nan_to_num(test)
  return np.mean(np.nanmean((gt - test)**2, axis=0))

zero_mses = []
mses = []

for i in range(0, 1):
    gt = np.loadtxt(GT_DIR + str(i) + '.txt')
    zero_mses.append(get_mse(gt, np.zeros_like(gt)))

    test = np.loadtxt(TEST_DIR+str(i)+".txt")

    mses.append(get_mse(gt, test))

percent_err_vs_all_zeros = 100*np.mean(mses)/np.mean(zero_mses)
print(percent_err_vs_all_zeros)
# print(f'YOUR ERROR SCORE IS {percent_err_vs_all_zeros:.2f}% (lower is better)')

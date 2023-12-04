#include "Utils/utils.h"
#include <windows.h>
#include <iostream>

// writeCSV函数需要忽略警告
#pragma warning(disable:4996)
#define _CRT_SECURE_NO_WARNINGS

void writeCSV(char* fileName, double* data, int colNum) {
	/*
		数据流写入CSV文件，单次时间步
		fileName: 数据文件路径，不存在自动创建
		data: double类型指针（仅限一维数据）
		colNum: 单次时间步的数据维数
	*/
	

	FILE* fpt;
	const char writeMode[] = "a+";
	fpt = fopen(fileName, writeMode);

	int i;
	bool isFirst = true;
	for (i = 0; i < colNum; i++)
	{
		if (!isFirst) fprintf(fpt, ",");
		fprintf(fpt, "%f", data[i]);
		isFirst = false;
	}
	fprintf(fpt, "\n");
	fclose(fpt);
}

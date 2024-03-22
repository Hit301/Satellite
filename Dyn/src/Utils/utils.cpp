#include "Utils/utils.h"
#include <windows.h>
#include <iostream>

// writeCSV������Ҫ���Ծ���
#pragma warning(disable:4996)
#define _CRT_SECURE_NO_WARNINGS

void writeCSV(char* fileName, double* data, int colNum) {
	/*
		������д��CSV�ļ�������ʱ�䲽
		fileName: �����ļ�·�����������Զ�����
		data: double����ָ�루����һά���ݣ�
		colNum: ����ʱ�䲽������ά��
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

#include "Utils/utils.h"
#include <windows.h>
#include <iostream>
#include <fstream>
#include <iomanip>

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


SaveDataClass::SaveDataClass(int precision)
{
	this->iter = 1;
	fout << std::setprecision(5);
	saveData.clear();
}


SaveDataClass::~SaveDataClass()
{
	this->fout.close();
}


void SaveDataClass::save(double *dataHead, int length)
{
	for (int i = 0; i < length; i++)
	{
		if (i != length - 1)
			this->fout << dataHead[i] <<',';
		else
			this->fout << dataHead[i] << std::endl;
	}
	
	for (auto const& x : this->saveData)
		this->fout << x[0] << ',' << x[1] << ',' << x[2] << '\n';

	fout.close(); // �����꼴�ر�
}

void SaveDataClass::load(std::vector<double> data)
{
	this->saveData.push_back(data);
}


void SaveDataClass::updateSaveFileName(char path[NAMA_LEN_MAX])
{
	fout.open(path);
}


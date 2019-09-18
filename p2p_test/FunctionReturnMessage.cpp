#include "stdafx.h"
#include "FunctionReturnMessage.h"
#include "string.h"

//���캯��
FunctionReturnMessage::FunctionReturnMessage()
{
	ResetAll();
}

//�������캯��
FunctionReturnMessage::FunctionReturnMessage(FunctionReturnMessage & value)
{
	bNoError = value.bNoError;
	iReturnCode = value.iReturnCode;

	memcpy(strErrorMessage, value.strErrorMessage, 128 * sizeof(char));
}

//����λ
void FunctionReturnMessage::ResetAll()
{
	bNoError = false;
	iReturnCode = 0;

	memset(strErrorMessage, '\0', 128 * sizeof(char));
}

//��Ϣ��λ
void FunctionReturnMessage::ResetMessage()
{
	memset(strErrorMessage, '\0', 128 * sizeof(char));
}

//������Ϣ
void FunctionReturnMessage::SetMessage(char * pMessage)
{
	//��������
	int iStringLength = strlen(pMessage);

	//У��
	if (0 > iStringLength) { return; }
	if (127 < iStringLength) { iStringLength = 127; }

	//����
	memcpy(strErrorMessage, pMessage, iStringLength*sizeof(char));
}
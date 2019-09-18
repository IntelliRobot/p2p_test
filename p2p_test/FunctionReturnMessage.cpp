#include "stdafx.h"
#include "FunctionReturnMessage.h"
#include "string.h"

//构造函数
FunctionReturnMessage::FunctionReturnMessage()
{
	ResetAll();
}

//拷贝构造函数
FunctionReturnMessage::FunctionReturnMessage(FunctionReturnMessage & value)
{
	bNoError = value.bNoError;
	iReturnCode = value.iReturnCode;

	memcpy(strErrorMessage, value.strErrorMessage, 128 * sizeof(char));
}

//对象复位
void FunctionReturnMessage::ResetAll()
{
	bNoError = false;
	iReturnCode = 0;

	memset(strErrorMessage, '\0', 128 * sizeof(char));
}

//信息复位
void FunctionReturnMessage::ResetMessage()
{
	memset(strErrorMessage, '\0', 128 * sizeof(char));
}

//设置信息
void FunctionReturnMessage::SetMessage(char * pMessage)
{
	//变量定义
	int iStringLength = strlen(pMessage);

	//校验
	if (0 > iStringLength) { return; }
	if (127 < iStringLength) { iStringLength = 127; }

	//拷贝
	memcpy(strErrorMessage, pMessage, iStringLength*sizeof(char));
}
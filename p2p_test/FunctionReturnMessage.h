#pragma once

class FunctionReturnMessage
{
public:
	bool bNoError;
	int  iReturnCode;

	char strErrorMessage[128];

public:
	FunctionReturnMessage();
	FunctionReturnMessage(FunctionReturnMessage & value);
	
	void ResetAll();
	void ResetMessage();
	void SetMessage(char * pMessage);
};

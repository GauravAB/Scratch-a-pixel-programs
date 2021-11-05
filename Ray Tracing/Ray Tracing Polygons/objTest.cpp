#include "objloader.h"


int main()
{
	bool ret = loadObjFile("african_head.obj");
	if (ret)
	{
		std::cout << "loading success\n";
	}

	return 0;
}
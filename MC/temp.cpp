#include <iostream>
#include <cstdio>

using namespace std;

int main()
{
    float s = 92.5 / 10;
    short j = (short)s;
    float t = s - j;

    printf("s: %f\n", s);
    cout << "j : " << j << endl;
    cout << "t : " << t << endl;
    

    return 0;
}
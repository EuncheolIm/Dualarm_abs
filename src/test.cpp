#include <thread>
#include <mutex>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <cstdlib>

using namespace Eigen;
using namespace std;

void func()
{
    static std::mutex m;

    m.lock();
    int a = 0;
    int b = 2;
    std::cout <<"a+b= "<< a+b << std::endl;
    m.unlock();
}
int main()
{
    func();
    Vector3d qq, p ;
    qq.setZero();
    p.setZero();

    // qq << -1, 2, 3;
    // cout << fabs(qq(0))<<endl;
    // cout << qq.cwiseAbs()<<endl;
    // p = qq.cwiseAbs();
    // cout <<p <<endl;
    int a, b,k;
    double x_range[10], q_range[10];

   a = 40;
   b = 10;
   if (b % 10 == 0)
   {
        cout << " a"<< a%10<< "  "<< b%10 <<endl;
   }
}

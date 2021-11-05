#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

#include "model.h"

Model::Model(const char* filename) : verts_(), faces_() ,sts_()
{
    std::ifstream in;
    in.open(filename, std::ifstream::in);

    if(in.fail()) {
        std::cerr << "file open failed Model::Model()" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string line;

    while(!in.eof())
    {
       std::getline(in,line);
       std::istringstream iss(line.c_str());
       char trash;

       if(!line.compare(0,2,"v "))
       {
           iss >> trash;

           Vec3f v;
           for(int i =0; i < 3; i++) { iss >> v[i];}
           verts_.push_back(v);
       }
       else if(!line.compare(0,3,"vt "))
       {
           iss >>trash;
           iss >>trash;
           Vec2f uv;
           for(int i =0; i < 2; i++) { iss >> uv[i];}
           sts_.push_back(uv);
       }
       else if(!line.compare(0,3,"vn "))
       {
           iss >>trash;
           iss >>trash;
           Vec3f n;
           for(int i =0; i < 3; i++) { iss >> n[i];}
           norms_.push_back(n);
       }
       else if(!line.compare(0,2,"f "))
       {
           iss >> trash;
           std::vector<int> face;
           int inorm, idx , uvx;
           //v/st/vn v/st/vn v/st/vn
           while(iss >> idx >> trash >> uvx >> trash >> inorm)
           {
               idx--;uvx--;inorm--;
               face.push_back(idx);
               face.push_back(uvx);
               face.push_back(inorm);
           }
           faces_.push_back(face);
       }
    }


    std::cerr << "# v# " << verts_.size() << " # sts# " << sts_.size() << " f# " << faces_.size()
    << " n# " << norms_.size() << std::endl;
}

Model::~Model()
{

}

int Model::nverts()
{
    return (int)verts_.size();
}

int Model::nfaces()
{
    return (int)faces_.size();
}


Vec3f Model::vert(int i)
{
    return verts_[i];
}

std::vector<int> Model::face(int i)
{
    return faces_[i];
}

Vec2f Model::st(int i)
{
    return sts_[i];
}

 Vec3f Model::norm(int index)
 {
     return norms_[index];
 }




